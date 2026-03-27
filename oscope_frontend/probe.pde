// ============================================================
//  Probe.pde — updated for raw uint16 (12-bit) stream
// ============================================================

class Probe {
  String       Label;
  float        minY, maxY;
  private GLayer Layer;
  GPointsArray   Data;
  color C;

  // ── TRIGGER STATE ──────────────────────────────────────────
  int     tLevel   = 2048;   // mid-scale for 12-bit
  int     tMode    = 0;
  boolean tEnabled = true;

  // ── DRAG ──────────────────────────────────────────────────
  boolean dragging = false;

  // ── TIMEOUT FALLBACK ──────────────────────────────────────
  int       noTriggerFrames = 0;
  final int MAX_HOLD        = 30;

  // ── HYSTERESIS ────────────────────────────────────────────
  int HYSTERESIS = 50;

  // ── FFT ───────────────────────────────────────────────────
  GPointsArray fftData;
  GLayer fftLayer;
  private int lastStartIdx = 0;

  // ─────────────────────────────────────────────────────────
  Probe(String name, color lineColor) {
    Label = name;
    Data  = new GPointsArray(displayResolution);
    for (int i = 0; i < displayResolution; i++) Data.add(new GPoint(i, 0));
    C = lineColor;
    plot.addLayer(Label, Data);
    Layer = plot.getLayer(Label);
    Layer.setLineColor(lineColor);
    Layer.setLineWidth(2.0);
    Layer.setPointSize(0);
    Layer.setPointColor(color(0, 0, 0, 0));

    fftData = new GPointsArray(displayResolution / 2);
    for (int i = 0; i < displayResolution / 2; i++) fftData.add(new GPoint(i, 0));

    minY = 0;
    maxY = 4095;   // 12-bit
  }

  // ── FIND TRIGGER ─────────────────────────────────────────
  int findTrigger(int available) {
    int searchLen = available - displayResolution;
    if (searchLen <= 0) return -1;

    int startIdx = (writePos - available + bufferCapacity) % bufferCapacity;

    for (int i = 1; i < searchLen; i++) {
      int prev = circularBuffer[(startIdx + i - 1) % bufferCapacity];
      int curr = circularBuffer[(startIdx + i)     % bufferCapacity];

      boolean rising  = (prev < tLevel - HYSTERESIS) && (curr >= tLevel);
      boolean falling = (prev > tLevel + HYSTERESIS) && (curr <= tLevel);

      if ((tMode == 0 && rising)  ||
          (tMode == 1 && falling) ||
          (tMode == 2 && (rising || falling))) {
        return (startIdx + i) % bufferCapacity;
      }
    }
    return -1;
  }

  // ── DRAW TIME-DOMAIN ─────────────────────────────────────
  void Draw() {
    int available = min(bufferCount, bufferCapacity);
    if (available < displayResolution) return;

    int startIdx;

    if (tEnabled) {
      int trigIdx = findTrigger(available);
      if (trigIdx == -1) {
        noTriggerFrames++;
        if (noTriggerFrames < MAX_HOLD) return;
        startIdx = (writePos - displayResolution + bufferCapacity) % bufferCapacity;
      } else {
        noTriggerFrames = 0;
        startIdx = trigIdx;
      }
    } else {
      noTriggerFrames = 0;
      startIdx = (writePos - displayResolution + bufferCapacity) % bufferCapacity;
    }

    lastStartIdx = startIdx;

    for (int i = 0; i < displayResolution; i++) {
      int   bufIdx = (startIdx + i) % bufferCapacity;
      float y      = circularBuffer[bufIdx];
      // x in milliseconds
      Data.set(i, (float)i / SAMPLING_RATE_HZ * 1000.0, y, "");
    }
    Layer.setPoints(Data);
  }

  // ── FFT ───────────────────────────────────────────────────
  void updateFFT(GPlot fp) {
    if (fp == null) return;

    int N = fftN;
    if (bufferCount < N) return;

    if (fftLayer == null) {
      int halfN = N / 2;
      fftData = new GPointsArray(halfN);
      for (int i = 0; i < halfN; i++) fftData.add(new GPoint(i, 0));
      fp.addLayer(Label + "_fft", fftData);
      fftLayer = fp.getLayer(Label + "_fft");
      fftLayer.setLineColor(C);
      fftLayer.setLineWidth(1.5);
      fftLayer.setPointSize(0);
      fftLayer.setPointColor(color(0, 0, 0, 0));
    }

    int startIdx = (writePos - N + bufferCapacity) % bufferCapacity;

    float[] re = new float[N];
    for (int i = 0; i < N; i++) {
      int   bufIdx = (startIdx + i) % bufferCapacity;
      float hann   = 0.5 * (1.0 - cos(TWO_PI * i / (N - 1)));
      re[i] = circularBuffer[bufIdx] * hann;
    }

    float[] im = new float[N];
    fft(re, im, N);

    int    half = N / 2;
    GPointsArray pts = new GPointsArray(half);

    for (int k = 0; k < half; k++) {
      float mag  = (float) Math.log10(max(1, sqrt(re[k]*re[k] + im[k]*im[k])));
      float freq = (float)k * SAMPLING_RATE_HZ / N;
      pts.add(new GPoint(freq, mag, ""));
    }

    fp.setYLim(0, 8);
    fftLayer.setPoints(pts);
    fftData = pts;
  }

  // ── COOLEY-TUKEY FFT ─────────────────────────────────────
  void fft(float[] re, float[] im, int N) {
    int j = 0;
    for (int i = 1; i < N; i++) {
      int bit = N >> 1;
      for (; (j & bit) != 0; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) {
        float tr = re[i]; re[i] = re[j]; re[j] = tr;
        float ti = im[i]; im[i] = im[j]; im[j] = ti;
      }
    }
    for (int len = 2; len <= N; len <<= 1) {
      float ang = -TWO_PI / len;
      float wRe = cos(ang), wIm = sin(ang);
      for (int i = 0; i < N; i += len) {
        float curRe = 1, curIm = 0;
        for (int k = 0; k < len / 2; k++) {
          float uRe = re[i+k],          uIm = im[i+k];
          float vRe = re[i+k+len/2] * curRe - im[i+k+len/2] * curIm;
          float vIm = re[i+k+len/2] * curIm + im[i+k+len/2] * curRe;
          re[i+k]       = uRe + vRe;  im[i+k]       = uIm + vIm;
          re[i+k+len/2] = uRe - vRe;  im[i+k+len/2] = uIm - vIm;
          float newRe = curRe*wRe - curIm*wIm;
          curIm = curRe*wIm + curIm*wRe;
          curRe = newRe;
        }
      }
    }
  }

  // ── TRIGGER LINE ─────────────────────────────────────────
  void DrawTriggerLine(GPlot p) {
    if (!tEnabled) return;

    float[] plotPos = p.getPos();
    float[] plotDim = p.getDim();
    float screenY = plotPos[1] + plotDim[1] -
                    map(tLevel, minY, maxY, 0, plotDim[1]);
    screenY = constrain(screenY, plotPos[1], plotPos[1] + plotDim[1]);

    stroke(255, 160, 40); strokeWeight(1.5);
    drawDashed(plotPos[0], screenY, plotPos[0] + plotDim[0], screenY, 10, 5);

    fill(255, 160, 40); noStroke();
    textSize(10); textAlign(LEFT, CENTER);
    text("T: " + tLevel, plotPos[0] + 4, screenY - 8);

    triangle(plotPos[0] - 12, screenY,
             plotPos[0] - 2,  screenY - 6,
             plotPos[0] - 2,  screenY + 6);
  }

  // ── DRAG ─────────────────────────────────────────────────
  boolean isOverHandle(GPlot p) {
    if (!tEnabled) return false;
    float[] plotPos = p.getPos();
    float[] plotDim = p.getDim();
    float screenY = plotPos[1] + plotDim[1] -
                    map(tLevel, minY, maxY, 0, plotDim[1]);
    return (mouseX < plotPos[0] + 10 && abs(mouseY - screenY) < 10);
  }

  void updateDrag(GPlot p) {
    float[] plotPos = p.getPos();
    float[] plotDim = p.getDim();
    float newLevel = map(mouseY,
                         plotPos[1] + plotDim[1], plotPos[1],
                         minY, maxY);
    tLevel = (int)constrain(newLevel, 0, 4095);

    if (cp5 != null) {
      Knob k = (Knob)cp5.getController("triggerLevel");
      if (k != null) k.setValue(tLevel);
    }
  }
}

// ── DASHED LINE HELPER ────────────────────────────────────────
void drawDashed(float x1, float y1, float x2, float y2,
                float dashLen, float gapLen) {
  float dx = x2 - x1, dy = y2 - y1;
  float total = sqrt(dx * dx + dy * dy);
  if (total == 0) return;
  float drawn = 0;
  float ux = dx / total, uy = dy / total;
  boolean drawing = true;
  while (drawn < total) {
    float segLen = drawing ? dashLen : gapLen;
    float end    = min(drawn + segLen, total);
    if (drawing) line(x1 + ux * drawn, y1 + uy * drawn,
                      x1 + ux * end,   y1 + uy * end);
    drawn   += segLen;
    drawing  = !drawing;
  }
}

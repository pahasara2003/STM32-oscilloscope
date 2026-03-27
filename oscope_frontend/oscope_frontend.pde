// ============================================================
//  GraficaDarkTheme.pde
// ============================================================

import grafica.*;
import processing.serial.*;
import controlP5.*;

// ── FFT SIZE ──────────────────────────────────────────────────
int fftN = 4096;

// ── CONTROLP5 ─────────────────────────────────────────────────
ControlP5 cp5;

// ── PLOTS ─────────────────────────────────────────────────────
GPlot plot;
GPlot fftPlot;

// ── DARK THEME COLORS ─────────────────────────────────────────
color BG        = color(18,  18,  28);
color PANEL_BG  = color(22,  22,  36);
color PLOT_BG   = color(28,  28,  42);
color GRID      = color(55,  55,  80);
color AXIS_LINE = color(180, 180, 210);
color LABEL_CLR = color(160, 160, 200);
color TITLE_CLR = color(220, 220, 255);
color ACCENT    = color(255, 160,  40);
color BTN_OFF   = color(38,  38,  60);
color BTN_HOV   = color(55,  55,  85);

color C1 = color(80,  200, 255);
color C2 = color(255, 120,  80);
color C3 = color(80,  255, 140);
color C4 = color(255, 240,  80);

// ── PROBE ─────────────────────────────────────────────────────
Probe probe1;

// ── SERIAL ────────────────────────────────────────────────────
Serial port;

// ── RAW UINT16 PARSER ─────────────────────────────────────────
// STM32 sends raw little-endian uint16 samples, no framing.
// We reassemble bytes into 16-bit samples here.
int    carry        = 0;
boolean hasCarry    = false;

// ── STATS ─────────────────────────────────────────────────────
int   framesReceived  = 0;
float measuredRateHz  = 500000.0;
long  statWindowStart = 0;
long  statSamples     = 0;

// ── CIRCULAR BUFFER ───────────────────────────────────────────
int[] circularBuffer;
int   writePos       = 0;
int   readPos        = 0;
int   bufferCount    = 0;
int   bufferCapacity = 65536;

int   displayResolution = 4*512;

float SAMPLING_RATE_HZ = 500000.0;
float SAMPLING_RATE    = SAMPLING_RATE_HZ;

// ── VIEW STATE ────────────────────────────────────────────────
int viewMode = 2;

// ── LAYOUT ────────────────────────────────────────────────────
final int TOOLBAR_H = 110;
final int MARGIN    = 14;
final int AXIS_PAD  = 44;
final int RIGHT_PAD = 18;

// ─────────────────────────────────────────────────────────────
void setup() {
  size(1800, 900);
  smooth();

  circularBuffer = new int[bufferCapacity];
  cp5 = new ControlP5(this);
  buildGUI();

  printArray(Serial.list());

  if (Serial.list().length > 0) {
    String portName = Serial.list()[0];
    port = new Serial(this, portName, 115200);
    port.clear();
    initPlots(portName);
  }

  statWindowStart = millis();
}

// ─────────────────────────────────────────────────────────────
void draw() {
  background(BG);
  drawToolbar();

  if (port == null) {
    fill(LABEL_CLR); noStroke();
    textSize(20); textAlign(CENTER, CENTER);
    text("Connect the STM32", width / 2, (height + TOOLBAR_H) / 2);
    return;
  }

  if (probe1 != null && probe1.dragging) probe1.updateDrag(plot);

  readSerial();

  if (probe1 != null) probe1.Draw();

  if (viewMode == 0 || viewMode == 2) {
    plot.beginDraw();
    plot.drawBackground();
    plot.drawBox();
    plot.drawXAxis();
    plot.drawYAxis();
    plot.drawTitle();
    plot.drawGridLines(GPlot.BOTH);
    plot.drawLines();
    if (probe1 != null) probe1.DrawTriggerLine(plot);
    plot.endDraw();
  }

  if (viewMode == 1 || viewMode == 2) {
    if (probe1 != null) probe1.updateFFT(fftPlot);
    fftPlot.beginDraw();
    fftPlot.drawBackground();
    fftPlot.drawBox();
    fftPlot.drawXAxis();
    fftPlot.drawYAxis();
    fftPlot.drawTitle();
    fftPlot.drawGridLines(GPlot.BOTH);
    fftPlot.drawLines();
    fftPlot.endDraw();
  }

  drawStatsOverlay();
  drawLegend();
}

// ─────────────────────────────────────────────────────────────
//  SERIAL PARSER
//  STM32 sends raw little-endian uint16 with no framing.
//  We carry over an odd byte between read() calls.
// ─────────────────────────────────────────────────────────────
void readSerial() {
  int available = port.available();
  if (available <= 0) return;

  byte[] raw = new byte[available];
  port.readBytes(raw);

  int i = 0;

  // consume any leftover byte from previous call
  if (hasCarry && raw.length > 0) {
    int lo = carry & 0xFF;
    int hi = raw[0] & 0xFF;
    pushSample(lo | (hi << 8));
    i = 1;
    hasCarry = false;
  }

  // consume pairs
  for (; i + 1 < raw.length; i += 2) {
    int lo = raw[i]     & 0xFF;
    int hi = raw[i + 1] & 0xFF;
    pushSample(lo | (hi << 8));
  }

  // save leftover odd byte
  if (i < raw.length) {
    carry    = raw[i] & 0xFF;
    hasCarry = true;
  }

  // measure sample rate every second
  long now     = millis();
  long elapsed = now - statWindowStart;
  if (elapsed >= 1000) {
    measuredRateHz   = statSamples * 1000.0 / elapsed;
    SAMPLING_RATE    = measuredRateHz;
    SAMPLING_RATE_HZ = measuredRateHz;
    statSamples      = 0;
    statWindowStart  = now;
    repositionPlots();
  }
}

void pushSample(int sample) {
  // clamp to 12-bit
  sample = constrain(sample, 0, 4095);

  circularBuffer[writePos] = sample;
  writePos = (writePos + 1) % bufferCapacity;
  if (bufferCount < bufferCapacity) bufferCount++;
  else readPos = (readPos + 1) % bufferCapacity;

  statSamples++;
  framesReceived++;
}

// ── TOOLBAR ───────────────────────────────────────────────────
void drawToolbar() {
  noStroke(); fill(PANEL_BG);
  rect(0, 0, width, TOOLBAR_H);
  stroke(GRID); strokeWeight(1);
  line(0, TOOLBAR_H, width, TOOLBAR_H);
}

// ── STATS OVERLAY ─────────────────────────────────────────────
void drawStatsOverlay() {
  int sx = width - 230;
  int sy = TOOLBAR_H + 10;

  noStroke(); fill(22, 22, 40, 215);
  rect(sx, sy, 215, 88, 6);

  textSize(11); noStroke();

  fill(LABEL_CLR); textAlign(LEFT, TOP);
  text("Samples:",       sx + 10, sy + 8);
  fill(C1);
  text(framesReceived,   sx + 110, sy + 8);

  fill(LABEL_CLR);
  text("Sample rate:",   sx + 10, sy + 26);
  fill(C3);
  text(nf(measuredRateHz / 1000.0, 0, 1) + " kSps", sx + 110, sy + 26);

  fill(LABEL_CLR);
  text("Buffer:",        sx + 10, sy + 44);
  fill(C4);
  text(nf(100.0 * bufferCount / bufferCapacity, 0, 1) + "%", sx + 110, sy + 44);

  fill(LABEL_CLR);
  text("Carry:",         sx + 10, sy + 62);
  fill(hasCarry ? color(255, 200, 80) : C3);
  text(hasCarry ? "1 byte" : "ok",  sx + 110, sy + 62);
}

// ─────────────────────────────────────────────────────────────
//  CONTROLP5 UI
// ─────────────────────────────────────────────────────────────
void buildGUI() {
  int bw = 108, bh = 32, gap = 8;
  int by = TOOLBAR_H / 2 - bh / 2;
  int x  = MARGIN;

  cp5.addButton("btnTime")
     .setLabel("  Time")
     .setPosition(x, by).setSize(bw, bh)
     .setColorBackground(BTN_OFF).setColorForeground(BTN_HOV)
     .setColorActive(color(80, 200, 255, 200))
     .setColorLabel(color(200, 230, 255));

  cp5.addButton("btnFFT")
     .setLabel("  FFT")
     .setPosition(x + bw + gap, by).setSize(bw, bh)
     .setColorBackground(BTN_OFF).setColorForeground(BTN_HOV)
     .setColorActive(color(80, 255, 140, 200))
     .setColorLabel(color(200, 255, 210));

  cp5.addButton("btnBoth")
     .setLabel("  Both")
     .setPosition(x + (bw + gap) * 2, by).setSize(bw, bh)
     .setColorBackground(BTN_OFF).setColorForeground(BTN_HOV)
     .setColorActive(color(255, 240, 80, 200))
     .setColorLabel(color(255, 245, 200));

  int tx = x + (bw + gap) * 3 + 20;

  cp5.addKnob("triggerLevel")
     .setLabel("Trig Level")
     .setRange(0, 4095).setValue(2048)   // 12-bit
     .setPosition(tx, 6).setSize(88, 88)
     .setDragDirection(Knob.VERTICAL)
     .setColorBackground(BTN_OFF)
     .setColorForeground(ACCENT)
     .setColorActive(color(255, 200, 80))
     .setColorLabel(LABEL_CLR)
     .setColorValueLabel(color(220, 220, 255));

  int rx = tx + 104;
  cp5.addRadioButton("triggerMode")
     .setPosition(rx, 16).setSize(20, 20).setSpacingRow(7)
     .setColorBackground(BTN_OFF)
     .setColorActive(ACCENT)
     .setColorForeground(color(70, 70, 100))
     .setColorLabel(LABEL_CLR)
     .addItem("Rise", 0)
     .addItem("Fall", 1)
     .addItem("Both", 2)
     .activate(0);

  int ex = rx + 105;
  cp5.addToggle("triggerEnabled")
     .setLabel("Trigger")
     .setPosition(ex, 20).setSize(54, 26)
     .setValue(true)
     .setColorBackground(color(55, 55, 75))
     .setColorActive(color(80, 255, 140))
     .setColorForeground(color(55, 80, 55))
     .setColorLabel(LABEL_CLR);

  int hx = ex + 78;
  cp5.addSlider("hysteresis")
     .setLabel("Hysteresis")
     .setRange(0, 200).setValue(50)   // 12-bit scale
     .setPosition(hx, 32).setSize(115, 16)
     .setColorBackground(BTN_OFF)
     .setColorForeground(C1)
     .setColorActive(color(120, 220, 255))
     .setColorLabel(LABEL_CLR)
     .setColorValueLabel(color(220, 220, 255));

  int fx = hx + 140;
  cp5.addScrollableList("fftSize")
     .setLabel("FFT Size")
     .setPosition(fx, by - 4).setSize(95, 130)
     .setBarHeight(bh).setItemHeight(26)
     .setColorBackground(BTN_OFF).setColorForeground(BTN_HOV)
     .setColorActive(color(255, 160, 40, 200))
     .setColorLabel(LABEL_CLR)
     .addItems(new String[]{"512", "1024", "2048", "4096", "8192"})
     .setValue(3)
     .close();
}

// ── CONTROLP5 CALLBACKS ───────────────────────────────────────
void btnTime()  { viewMode = 0; repositionPlots(); }
void btnFFT()   { viewMode = 1; repositionPlots(); }
void btnBoth()  { viewMode = 2; repositionPlots(); }

void triggerLevel(int val) {
  if (probe1 != null) probe1.tLevel = val;
}
void triggerMode(int val) {
  if (probe1 != null) probe1.tMode = val;
}
void triggerEnabled(boolean val) {
  if (probe1 != null) probe1.tEnabled = val;
}
void hysteresis(float val) {
  if (probe1 != null) probe1.HYSTERESIS = (int)val;
}
void fftSize(int index) {
  int[] sizes = {512, 1024, 2048, 4096, 8192};
  fftN = sizes[index];
  if (probe1 != null) probe1.fftLayer = null;
}

// ─────────────────────────────────────────────────────────────
//  PLOT INIT & LAYOUT
// ─────────────────────────────────────────────────────────────
void initPlots(String portName) {
  plot    = new GPlot(this);
  fftPlot = new GPlot(this);
  probe1  = new Probe("probe1", C1);

  stylePlot(plot,    "Time Domain — " + portName, "time [ms]", "ADC (12-bit)");
  stylePlot(fftPlot, "FFT Spectrum",               "freq [Hz]", "magnitude [dB]");

  repositionPlots();
}

void repositionPlots() {
  if (plot == null) return;

  int py     = TOOLBAR_H + MARGIN;
  int ph     = height - py - MARGIN - 100;
  int totalW = width - MARGIN * 2;

  switch (viewMode) {
    case 0:
      plot.setPos(20, py);
      plot.setDim(totalW - 120, ph);
      break;
    case 1:
      fftPlot.setPos(20, py);
      fftPlot.setDim(totalW - 120, ph);
      break;
    case 2:
      int half = (totalW - AXIS_PAD * 2 - RIGHT_PAD - 100) / 2;
      plot.setPos(20, py);
      plot.setDim(half, ph);
      fftPlot.setPos(half + 116, py);
      fftPlot.setDim(half, ph);
      break;
  }

  float windowMs = (displayResolution / SAMPLING_RATE_HZ) * 1000.0;
  plot.setXLim(0, windowMs);
  plot.setYLim(0, 4095);   // 12-bit

  float nyquist = SAMPLING_RATE_HZ / 2.0;
  fftPlot.setXLim(0, nyquist);
  fftPlot.setYLim(0, 8);
}

void stylePlot(GPlot p, String title, String xLabel, String yLabel) {
  p.setBgColor(PLOT_BG);
  p.setBoxBgColor(PLOT_BG);
  p.setBoxLineColor(AXIS_LINE);
  p.setBoxLineWidth(1.5);
  p.setGridLineColor(GRID);
  p.setGridLineWidth(1);

  p.getTitle().setText(title);
  p.getTitle().setTextAlignment(CENTER);
  p.getTitle().setFontColor(TITLE_CLR);
  p.getTitle().setFontSize(15);
  p.getTitle().setOffset(16);

  p.getXAxis().getAxisLabel().setText(xLabel);
  p.getXAxis().getAxisLabel().setFontColor(LABEL_CLR);
  p.getXAxis().getAxisLabel().setFontSize(12);

  p.getYAxis().getAxisLabel().setText(yLabel);
  p.getYAxis().getAxisLabel().setFontColor(LABEL_CLR);
  p.getYAxis().getAxisLabel().setFontSize(12);

  styleAxis(p.getXAxis());
  styleAxis(p.getYAxis());
  styleAxis(p.getTopAxis());
  styleAxis(p.getRightAxis());
}

// ── MOUSE ─────────────────────────────────────────────────────
void mousePressed() {
  if (probe1 != null && !cp5.isMouseOver())
    probe1.dragging = probe1.isOverHandle(plot);
}
void mouseReleased() {
  if (probe1 != null) probe1.dragging = false;
}

// ── LEGEND ────────────────────────────────────────────────────
void drawLegend() {
  if (plot == null || viewMode == 1) return;
  float[] pos = plot.getPos();
  float[] dim = plot.getDim();

  float lx = pos[0] + dim[0] - 100;
  float ly = pos[1] + 80;

  noStroke(); fill(32, 32, 52, 215); rect(lx, ly, 148, 38, 5);
  stroke(C1); strokeWeight(2.5); noFill();
  line(lx + 8, ly + 19, lx + 28, ly + 19);
  fill(LABEL_CLR); noStroke(); textSize(11); textAlign(LEFT, CENTER);
  text("Channel 01", lx + 34, ly + 19);
}

// ── AXIS HELPER ───────────────────────────────────────────────
void styleAxis(GAxis ax) {
  ax.setFontColor(LABEL_CLR);
  ax.setFontSize(11);
  ax.setLineColor(AXIS_LINE);
  ax.setTickLength(4);
}

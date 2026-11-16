// Web Serial link (Chrome / Edge 89+): USB-UART adapter wired to the HC-12.
// Same interface as VirtualLink: open(), write(bytes), close(), onData, onClose.

const USB_NAMES = {
  0x1A86: 'WCH CH340/CH341', 0x10C4: 'Silicon Labs CP210x', 0x0403: 'FTDI', 0x067B: 'Prolific PL2303',
  0x2341: 'Arduino', 0x2A03: 'Arduino', 0x239A: 'Adafruit', 0x303A: 'Espressif',
};

const hex4 = (v) => v.toString(16).toUpperCase().padStart(4, '0');

export class WebSerialLink {
  static supported() {
    return typeof navigator !== 'undefined' && 'serial' in navigator;
  }

  constructor({ baud = 9600, port = null } = {}) {
    this.kind = 'serial';
    this.baud = baud;
    this.port = port;
    this.name = 'Serial port';
    this.isOpen = false;
    this.onData = null;
    this.onClose = null;
    this.onError = null;
    this.reader = null;
    this.writer = null;
    this._loop = null;
  }

  async open() {
    if (!WebSerialLink.supported()) {
      throw new Error('Web Serial is not available in this browser. Use Chrome or Edge on a desktop/tablet ' +
        '(page opened from file:// or http://localhost), or use the virtual rover.');
    }
    if (!this.port) this.port = await navigator.serial.requestPort(); // needs a user gesture
    await this.port.open({ baudRate: this.baud, dataBits: 8, stopBits: 1, parity: 'none', flowControl: 'none',
      bufferSize: 8192 });
    const info = this.port.getInfo ? this.port.getInfo() : {};
    if (info.usbVendorId) {
      this.name = `${USB_NAMES[info.usbVendorId] || 'USB serial'} (${hex4(info.usbVendorId)}:${hex4(info.usbProductId || 0)})`;
    }
    this.writer = this.port.writable.getWriter();
    this.isOpen = true;
    this._onDisconnect = (e) => { if (e.target === this.port || e.port === this.port) this._lost('device unplugged'); };
    navigator.serial.addEventListener('disconnect', this._onDisconnect);
    this._loop = this._readLoop();
  }

  async _readLoop() {
    while (this.isOpen && this.port.readable) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value && value.length && this.onData) this.onData(value);
        }
      } catch (e) {
        // BreakError / FramingError / ParityError / BufferOverrunError are recoverable:
        // the stream is re-created and we keep reading. NetworkError = device gone.
        if (this.onError) this.onError(e);
        if (e && e.name === 'NetworkError') { this.reader.releaseLock(); this.reader = null; break; }
      } finally {
        if (this.reader) { try { this.reader.releaseLock(); } catch (e) { /* ignore */ } }
        this.reader = null;
      }
    }
    if (this.isOpen) this._lost('serial stream ended');
  }

  write(bytes) {
    if (!this.isOpen || !this.writer) return Promise.reject(new Error('serial port closed'));
    return this.writer.write(bytes);
  }

  async _release() {
    this.isOpen = false;
    try { if (this.reader) await this.reader.cancel(); } catch (e) { /* ignore */ }
    try { if (this._loop) await this._loop; } catch (e) { /* ignore */ }
    try { if (this.writer) { this.writer.releaseLock(); this.writer = null; } } catch (e) { /* ignore */ }
    try { await this.port.close(); } catch (e) { /* ignore */ }
    if (this._onDisconnect) navigator.serial.removeEventListener('disconnect', this._onDisconnect);
  }

  async close() {
    if (!this.isOpen) return;
    await this._release();
    if (this.onClose) this.onClose('closed');
  }

  async _lost(reason) {
    if (!this.isOpen) return;
    this.isOpen = false;
    if (this.onClose) this.onClose(reason);
    try { if (this.writer) { this.writer.releaseLock(); this.writer = null; } } catch (e) { /* ignore */ }
    try { await this.port.close(); } catch (e) { /* ignore */ }
    if (this._onDisconnect) navigator.serial.removeEventListener('disconnect', this._onDisconnect);
  }
}

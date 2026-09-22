import { Transport, ESPLoader } from "esptool-js";

function sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function flashFirmware(firmwareBytes, onEvent) {
  if (!("serial" in navigator)) {
    throw new Error("unsupported");
  }
  if (!window.isSecureContext) {
    throw new Error("not-allowed");
  }

  const port = await navigator.serial.requestPort();

  const terminal = {
    clean() {},
    write(str) {
      onEvent({ type: "log", message: str });
    },
    writeLine(str) {
      onEvent({ type: "log", message: str });
    },
  };

  const transport = new Transport(port);
  const loader = new ESPLoader({
    transport,
    baudrate: 115200,
    romBaudrate: 115200,
    terminal,
  });

  onEvent({ type: "state", state: "connecting" });
  await loader.main();

  onEvent({ type: "state", state: "writing" });
  await loader.writeFlash({
    fileArray: [{ data: firmwareBytes, address: 0 }],
    flashSize: "keep",
    eraseAll: false,
    compress: true,
    reportProgress(fileIndex, written, total) {
      onEvent({ type: "progress", written, total });
    },
  });

  onEvent({ type: "state", state: "resetting" });
  await transport.setRTS(true);
  await sleep(100);
  await loader.after();
  await transport.disconnect();

  onEvent({ type: "state", state: "done" });
}

window.ClotheslineFlasher = { flashFirmware };

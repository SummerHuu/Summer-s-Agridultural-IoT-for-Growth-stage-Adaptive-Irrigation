// ==== Firebase Config ====
const firebaseConfig = {
  apiKey: "AIzaSyB8pqm5yKESLbNxbU4A11xSoLve2nCnH54",
  authDomain: "summer-s-agricultural-iot.firebaseapp.com",
  databaseURL:
    "https://summer-s-agricultural-iot-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "summer-s-agricultural-iot",
  storageBucket: "summer-s-agricultural-iot.firebasestorage.app",
  messagingSenderId: "721765143443",
  appId: "1:721765143443:web:2b979107ea8c0f43b84e66",
  measurementId: "G-H4FG10VX2L",
};

// Initialize Firebase
const app = firebase.initializeApp(firebaseConfig);
const db = firebase.database();

// ==== Utility ====
function updateElementText(id, value, suffix = "") {
  const el = document.getElementById(id);
  if (el) el.innerText = value + suffix;
}

// camera
db.ref("Camera/ip").on("value", (snap) => {
  const ip = snap.val();
  document.querySelector(".video-feed").href = `http://${ip}`;
  console.log(`http://${ip}`);
});

// Listen for changes in /Camera/growth index
db.ref("Camera/growth index").on("value", (snap) => {
  const newValue = snap.val();
  console.log(`[Listener] Detected new growth index: ${newValue}`);
  // Write to /Pump/command
  db.ref("Pump/command")
    .set(newValue)
    .then(() => console.log(`[Update] Pump/command updated to: ${newValue}`))
    .catch((error) =>
      console.error(`[Error] Failed to update Pump/command:`, error)
    );
});

// BLE
// db.ref("Camera/BLE").on("value", (snap) => {
//   updateElementText("BLE", snap.val());
// });
db.ref("Pump/quota").on("value", (snap) => {
  updateElementText("quota", snap.val());
});

// ==== assessment ====
db.ref("Decision/automation").on("value", (snap) => {
  updateElementText(
    "automationMode",
    snap.val() ? "Automatic Decision" : "Manual Decision"
  );
});

db.ref("Decision/decision").on("value", (snap) => {
  updateElementText("decision", snap.val());
});

db.ref("Camera/growing stage").on("value", (snap) => {
  updateElementText("growing stage", snap.val());
});

db.ref("Sensors/soil quality").on("value", (snap) => {
  updateElementText("soil quality", snap.val());
});

// ==== Realtime Sensor Listeners ====
db.ref("Sensors/temperature").on("value", (snap) => {
  updateElementText("temperature", snap.val(), " °C");
});

db.ref("Sensors/humidity").on("value", (snap) => {
  updateElementText("humidity", snap.val(), " %");
});

db.ref("Sensors/moisture").on("value", (snap) => {
  updateElementText("moisture", snap.val(), " %");
});

// ==== Flowmeter Readings ====
db.ref("Flowmeter/flow rate").on("value", (snap) => {
  updateElementText("flowRate", snap.val(), " L/min");
});

db.ref("Flowmeter/irrigation times").on("value", (snap) => {
  updateElementText("irrigationTimes", snap.val(), " times");
});

db.ref("Flowmeter/single millilitter").on("value", (snap) => {
  updateElementText("singleMl", snap.val(), " mL");
});

db.ref("Flowmeter/total millilitter").on("value", (snap) => {
  updateElementText("totalMl", snap.val(), " mL");
});

// ==== Robot Control ====
function sendDirectionCommand(cmd) {
  db.ref("Robot/direction").set(cmd);
  console.log("Direction:", cmd);
}

function sendTiltCommand(cmd) {
  db.ref("Robot/tilt").set(cmd);
  console.log("Tilt:", cmd);
}

// Movement Control
function startDirectionCommand(cmd, el) {
  sendDirectionCommand(cmd);
  if (el) el.classList.add("active");
}

function stopDirectionCommand(el) {
  sendDirectionCommand("S");
  if (el) el.classList.remove("active");
}

// Tilt Control
function startTiltCommand(cmd, el) {
  sendTiltCommand(cmd);
  if (el) el.classList.add("active");
}

function stopTiltCommand(el) {
  sendTiltCommand("");
  if (el) el.classList.remove("active");
}

// ==== Irrigation Control ====
function startIrrigationCommand(cmd, el) {
  db.ref("Pump/command").set(cmd);
  console.log("Pump:", cmd);
  if (el) el.classList.add("active");
}

function stopIrrigationCommand(el) {
  db.ref("Pump/command").set("");
  console.log("Pump:", "");
  if (el) el.classList.remove("active");
}

// quota
function sendIrrigationQuota(type = "#", el = null) {
  if (el) el.classList.add("active"); // only if button element exists

  if (type === "#") {
    const input = document.getElementById("irrigationAmount");
    const amount = parseInt(input.value);
    if (!isNaN(amount) && amount > 0) {
      startIrrigationCommand(`#${amount}`, el);
    } else {
      alert("Please enter a valid irrigation amount in mL.");
      if (el) el.classList.remove("active"); // rollback animation on error
    }
  } else {
    // other types if needed
  }
}
function endIrrigationQuota(type, el) {
  stopIrrigationCommand(el);
  el.classList.remove("active");
}

// ==== Keyboard Support ====
const keyBindings = {
  ArrowUp: { type: "direction", command: "F" },
  ArrowDown: { type: "direction", command: "B" },
  ArrowLeft: { type: "direction", command: "L" },
  ArrowRight: { type: "direction", command: "R" },
  S: { type: "direction", command: "S" },
  U: { type: "tilt", command: "U" },
  D: { type: "tilt", command: "D" },
};

let keyPressed = {};

window.addEventListener(
  "keydown",
  (e) => {
    const key = e.key.length === 1 ? e.key.toUpperCase() : e.key; // Normalize key
    const binding = keyBindings[key];

    // Prevent scrolling for arrow keys only
    if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"].includes(e.key)) {
      e.preventDefault();
    }

    if (!binding || keyPressed[key]) return;

    keyPressed[key] = true;

    if (binding.type === "direction") {
      sendDirectionCommand(binding.command);
      highlightButtonByCommand(binding.command);
    } else if (binding.type === "tilt") {
      sendTiltCommand(binding.command);
      highlightTiltButton(binding.command);
    }
  },
  { capture: true, passive: false } // prevent scroll-on-hold for arrows
);

window.addEventListener(
  "keyup",
  (e) => {
    const key = e.key.length === 1 ? e.key.toUpperCase() : e.key;
    const binding = keyBindings[key];
    keyPressed[key] = false;

    if (!binding) return;

    if (binding.type === "direction") {
      sendDirectionCommand("S");
    } else if (binding.type === "tilt") {
      sendTiltCommand("");
    }

    removeHighlight();
  },
  { capture: true, passive: false } // prevent scroll-on-hold for arrows
);

// ==== Button Highlight ====
function highlightButtonByCommand(cmd) {
  const allButtons = document.querySelectorAll("button");
  allButtons.forEach((btn) => {
    if (
      (cmd === "F" && btn.textContent.includes("⬆")) ||
      (cmd === "B" && btn.textContent.includes("⬇")) ||
      (cmd === "L" && btn.textContent.includes("⬅")) ||
      (cmd === "R" && btn.textContent.includes("➡")) ||
      (cmd === "S" && btn.textContent.includes("⏹"))
    ) {
      btn.classList.add("active");
    }
  });
}

function highlightTiltButton(cmd) {
  const allButtons = document.querySelectorAll("button");
  allButtons.forEach((btn) => {
    if (
      (cmd === "U" && btn.textContent.toLowerCase().includes("tilt up")) ||
      (cmd === "D" && btn.textContent.toLowerCase().includes("tilt down"))
    ) {
      btn.classList.add("active");
    }
  });
}

function removeHighlight() {
  document
    .querySelectorAll("button.active")
    .forEach((btn) => btn.classList.remove("active"));
}

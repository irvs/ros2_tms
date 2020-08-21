// DRDoubleSDK is a global object loaded in by Electron in the Standby window and a "Trusted" Accessory window
if (!("DRDoubleSDK" in window)) {
	console.error("window.DRDoubleSDK not found. This is required.");
}

function q(selector) {
	return document.querySelector(selector);
}

// Utilities

function degToRad(degrees) {
	return degrees * Math.PI / 180;
}
function radToDeg(radians) {
	return radians * 180 / Math.PI;
}
function inchesToMeters(inches) {
	return inches * .0254;
}
window.degToRad = degToRad;
window.radToDeg = radToDeg;
window.inchesToMeters = inchesToMeters;

var posX = 0;
var posY = 0;
var posYaw = 0;
var speedX = 0;

// ************************************************************
// Log
function log(text) {
  console.log(text);
  q("#message").innerText = text;
  //q.innerHTML += "<div>" + text + "</div>";
}

var socket = null;
function connectWebsocket() {
  socket = new WebSocket("wss://" + window.location.hostname);
  //socket = new WebSocket("wss://somber-persistent-line.glitch.me");
  socket.onopen = function(event) { log("Connected to server: " + window.location.hostname); };
  socket.onclose = function() {
    log("Disconnected from server");
    endCall();
    socket = null;
    setTimeout(connectWebsocket, 1000);
  }

  socket.onmessage = function(event) {
    var signal = null;
    try {
      signal = JSON.parse(event.data);
      log(signal);
    } catch (e) {
      log(event.data);
    }

    if (signal) {
      // Note: DO NOT pass DRDoubleSDK commands directy from your driver or you will be opening a massive security hole to your robot. You should use your own command structure for your signaling server, then hard-code commands for the DRDoubleSDK on the robot side - just like we're doing right here.
      switch (signal.type) {

        case "isRobotAvailable":
          sendToServer({ type: "robotIsAvailable", message: "Robot is here!" });
          log("Received availability check");
          break;

        case "preheat":
          DRDoubleSDK.sendCommand("camera.enable", { template: "preheat" });
          break;
          
        case "startCall":
          log("startCall");
          DRDoubleSDK.sendCommand("webrtc.enable", {
            servers: signal.servers,
            transportPolicy: signal.transportPolicy || "all",
            manageCamera: true
          });
          break;

        case "endCall":
          endCall();
          break;

        case "answer":
        case "candidate":
          log("Received signal");
          DRDoubleSDK.sendCommand("webrtc.signal", signal);
          break;

        case "poleStand":
          DRDoubleSDK.sendCommand("base.pole.stand");
          break;

        case "poleSit":
          DRDoubleSDK.sendCommand("base.pole.sit");
          break;

        case "poleStop":
          DRDoubleSDK.sendCommand("base.pole.stop");
          break;
          
        case "enableNavigation":
          DRDoubleSDK.sendCommand("navigate.enable");
          break;

        case "relativeTarget":
          if (signal.hasOwnProperty("x") && signal.hasOwnProperty("y")) {
            DRDoubleSDK.sendCommand("navigate.target", { relative: true, x: signal.x, y: signal.y });
          }
          break;
      }
    }
  };
}
connectWebsocket();

function sendToServer(message) {
  socket.send(JSON.stringify(message));
}

function endCall() {
  log("endCall");
  DRDoubleSDK.sendCommand("webrtc.disable");
  DRDoubleSDK.sendCommand("navigate.disable");
}

// DRDoubleSDK is preloaded in the web view on the robot, so it will show errors on the Glitch.com editor
if (DRDoubleSDK == "undefined") {
  var DRDoubleSDK = {};
}

// Make sure the camera and webrtc modules are off, so we can use them.
DRDoubleSDK.sendCommand("camera.disable");
DRDoubleSDK.sendCommand("webrtc.disable");
/*
// We must reset the watchdog faster than every 3 seconds, so D3 knows that our pages is still running ok.
DRDoubleSDK.resetWatchdog();
window.setInterval(() => {
  DRDoubleSDK.resetWatchdog();
  DRDoubleSDK.sendCommand("screensaver.nudge");
}, 2000);

DRDoubleSDK.sendCommand("events.subscribe", {
  events: [
    "DRWebRTC.signal"
  ]
});
*/

DRDoubleSDK.on("event", (message) => {
	// Event messages include: { class: "DRNetwork", key: "info", data: {...} }
	switch (message.class + "." + message.key) {

		// DRNetwork
		case "DRNetwork.info": {
			q("#wifi_ssid").innerText = (message.data.connection == "connected" && message.data.ssid) ? message.data.ssid : "Unknown";
			q("#wifi_ip").innerText = message.data.internalIp;
			break;
		}

		case "DRBase.status": {
			break;
		}

		case "DRPose.pose": {
			q("#pose").innerText = "(" + message.data.base.x.toFixed(3) + " " + message.data.base.y.toFixed(3) + " " + radToDeg(message.data.base.yaw).toFixed(1) + " v " + message.data.base.speed + ")";
                        posX = message.data.base.x;
                        posY = message.data.base.y;
                        posYaw = message.data.base.yaw;
			speedX = message.data.base.speed;
			break;
		}

		case "DRWebRTC.signal": {
			sendToServer(message.data);
			break;
		}
	}
});

function onConnect() {
	if (DRDoubleSDK.isConnected()) {
		DRDoubleSDK.resetWatchdog();
		// Subscribe to events that you will process. You can subscribe to more events at any time.
		DRDoubleSDK.sendCommand("events.subscribe", {
			events: [
				"DRBase.status",
				"DRNetwork.info",
				"DRPose.pose",
				"DRWebRTC.signal"
			]
		});

		// Send commands any time – here, we're requesting initial info to show
		DRDoubleSDK.sendCommand("network.requestInfo");
		DRDoubleSDK.sendCommand("base.requestStatus");

		// Turn on the screen, but allow the screensaver to kick in later
		DRDoubleSDK.sendCommand("screensaver.nudge");

	} else {
		window.setTimeout(onConnect, 100);
	}
}

function start() {
	// REQUIRED: Tell d3-api that we're still running ok (faster than every 3000 ms) or the page will be reloaded.
	window.setInterval(() => {
		DRDoubleSDK.resetWatchdog();
	}, 2000);

	// DRDoubleSDK 
	onConnect();
	DRDoubleSDK.on("connect", () => {
		onConnect();
	});
};

window.onload = () => {
	// REQUIRED: Tell d3-api that we're still running ok (faster than every 3000 ms) or the page will be reloaded.
	window.setInterval(() => {
		DRDoubleSDK.resetWatchdog();
	}, 2000);

	// DRDoubleSDK 
	onConnect();
	DRDoubleSDK.on("connect", () => {
		onConnect();
	});
};

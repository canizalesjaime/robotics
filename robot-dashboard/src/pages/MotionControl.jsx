import React from "react";
import CameraView from "./CameraView";
import { API_URL } from "./config";

export default function MotionControl() {
  const [speed, setSpeed] = React.useState("");
  const activeKeyRef = React.useRef(null); // prevents repeat firing

  const sendCommand = (action) => {
    fetch(API_URL+"/command", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ action }),
    });
  };

  const changeSpeed = () => {
    const value = Number(speed);

    if (value < 0 || value > 100 || isNaN(value)) {
      alert("Speed must be between 0 and 100");
      return;
    }

    fetch(API_URL+"/speed", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify({ speed: value }),
    });
  };

  /* ---------------- Keyboard Controls ---------------- */

  React.useEffect(() => {
    const handleKeyDown = (e) => {
      if (activeKeyRef.current) return; // already moving

      switch (e.key) {
        case "ArrowUp":
          sendCommand("forward");
          activeKeyRef.current = "ArrowUp";
          break;
        case "ArrowDown":
          sendCommand("backward");
          activeKeyRef.current = "ArrowDown";
          break;
        case "ArrowLeft":
          sendCommand("left");
          activeKeyRef.current = "ArrowLeft";
          break;
        case "ArrowRight":
          sendCommand("right");
          activeKeyRef.current = "ArrowRight";
          break;
        default:
          break;
      }
    };

    const handleKeyUp = (e) => {
      if (e.key === activeKeyRef.current) {
        sendCommand("stop");
        activeKeyRef.current = null;
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, []);

  /* --------------------------------------------------- */

  return (
    <div className="space-y-6">
      <h1 className="text-2xl font-bold">Motion Control</h1>

      {/* Speed control */}
      <div className="flex items-center gap-2 mt-4 justify-center">
        <input
          type="number"
          min="0"
          max="100"
          value={speed}
          onChange={(e) => setSpeed(e.target.value)}
          className="border rounded px-2 py-1 w-20 text-center"
          placeholder="0-100"
        />

        <button
          onClick={changeSpeed}
          className="bg-blue-500 text-white px-3 py-1 rounded"
        >
          Change Speed
        </button>
      </div>

      {/* Motion buttons */}
      <div className="grid grid-cols-3 gap-4 w-48 mx-auto">
        <button
          onMouseDown={() => sendCommand("forward")}
          onMouseUp={() => sendCommand("stop")}
          className="bg-green-500 text-white p-2 rounded"
        >
          ↑
        </button>

        <button
          onMouseDown={() => sendCommand("left")}
          onMouseUp={() => sendCommand("stop")}
          className="bg-yellow-500 text-white p-2 rounded"
        >
          ←
        </button>

        <button
          onMouseDown={() => sendCommand("right")}
          onMouseUp={() => sendCommand("stop")}
          className="bg-yellow-500 text-white p-2 rounded"
        >
          →
        </button>

        <button
          onMouseDown={() => sendCommand("backward")}
          onMouseUp={() => sendCommand("stop")}
          className="bg-red-500 text-white p-2 rounded col-span-3"
        >
          ↓
        </button>
      </div>

      <CameraView />
    </div>
  );
}

import React, { useState } from "react";
import { API_URL } from "./config";

export default function ArmControl() {
  const [angles, setAngles] = useState({
    base: 90,
    shoulder: 90,
    elbow: 90,
    gripper: 90,
  });

  const handleChange = (joint, value) => {
    const clamped = Math.max(0, Math.min(180, Number(value)));
    setAngles({ ...angles, [joint]: clamped });
  };

  const sendAngles = () => {
    fetch(API_URL+"/set_angles", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
      },
      body: JSON.stringify(angles),
    });
  };

  return (
    <div>
      <h1 className="text-2xl font-bold mb-4">Arm Control</h1>

      <div className="flex flex-col gap-3 max-w-xs">
        {["base", "shoulder", "elbow", "gripper"].map((joint) => (
          <div key={joint} className="flex justify-between items-center">
            <label className="capitalize">{joint}</label>
            <input
              type="number"
              min="0"
              max="180"
              value={angles[joint]}
              onChange={(e) => handleChange(joint, e.target.value)}
              className="border p-1 w-20"
            />
          </div>
        ))}

        <button
          onClick={sendAngles}
          className="bg-blue-500 text-white p-2 rounded mt-3"
        >
          Send Angles
        </button>
      </div>
    </div>
  );
}

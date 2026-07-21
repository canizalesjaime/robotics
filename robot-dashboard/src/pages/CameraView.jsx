import React from "react";
import { API_URL } from "./config";

export default function CameraView() {
  return (
    <div className="space-y-4">
      <h1 className="text-2xl font-bold">Live Camera</h1>

      <div className="border rounded overflow-hidden">
        <img
          src={`${API_URL}/camera`}
          alt="Robot Camera"
          className="w-full"
        />
      </div>
    </div>
  );
}

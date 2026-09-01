import React, { useState, useEffect } from "react";
import { API_URL } from "./config";


export default function Sensors() {
  const [data, setData] = useState({ x: 0, y: 0, z: 0 });

  useEffect(() => {
    const interval = setInterval(() => {
      fetch(API_URL + "/robot_state",{cache: "no-store"})
  .then(res => res.json())
  .then(state => {

    setData({
      x: state.imu.roll,
      y: state.imu.pitch,
      z: state.imu.yaw
    });
  });

    }, 500);

    return () => clearInterval(interval);
  }, []);


  return (
    <div>
      <h1 className="text-2xl font-bold mb-4">Sensors Data</h1>
      <h2 className="text-2xl font-bold mb-4"> Accelerometer:</h2>
      <p>Roll: {data.x}</p>
      <p>Pitch: {data.y}</p>
      <p>Yaw: {data.z}</p>
    </div>
  );
}
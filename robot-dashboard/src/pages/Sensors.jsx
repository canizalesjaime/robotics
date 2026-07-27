import React, { useState, useEffect } from "react";
import { API_URL } from "./config";


export default function Sensors() {
  const [data, setData] = useState({ x: 0, y: 0, z: 0 });
  const [lidar, setLidar] = useState({ distance: 0, strength: 0 });

  useEffect(() => {
    const interval = setInterval(() => {
      fetch(API_URL+"/accelerometer")
        .then((res) => res.json())
        .then(setData);

      fetch(API_URL+"/lidar")
        .then((res) => res.json())
        .then(setLidar);

    }, 500);

    return () => clearInterval(interval);
  }, []);

  const rotateBase = async () => {
    await fetch(API_URL+"/rotate_base", {method: "POST"});
  };

  const stopBase = async () => {
    await fetch(API_URL+"/stop_base", {method: "POST"});
  };

  return (
    <div>
      <h1 className="text-2xl font-bold mb-4">Sensors Data</h1>
      <h2 className="text-2xl font-bold mb-4"> Accelerometer:</h2>
      <p>Roll: {data.x}</p>
      <p>Pitch: {data.y}</p>
      <p>Temp: {data.z}</p>

      <h2 className="text-2xl font-bold mb-4"> Lidar:</h2>
      <p className="mt-4">Distance: {lidar.distance} cm</p>
      <p>Strength: {lidar.strength}</p>

      <div className="mt-4 space-x-2">
        <button onClick={rotateBase} className="bg-blue-500 text-white px-4 py-2 rounded">
          Rotate Base
        </button>

        <button onClick={stopBase} className="bg-red-500 text-white px-4 py-2 rounded">
          Stop Base
        </button>
      </div>
    </div>
  );
}
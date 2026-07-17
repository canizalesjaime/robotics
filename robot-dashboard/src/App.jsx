import React from "react";
import { Routes, Route } from "react-router-dom";
import Navbar from "./components/Navbar";
import MotionControl from "./pages/MotionControl";
import ArmControl from "./pages/ArmControl";
import Sensors from "./pages/Sensors";
import AboutMe from "./pages/AboutMe";

export default function App() {
  return (
    <div>
      <Navbar />
      <main className="pt-20 max-w-7xl mx-auto px-6">
        <Routes>
          <Route path="/" element={<MotionControl />} />
          <Route path="/arm" element={<ArmControl />} />
          <Route path="/sensors" element={<Sensors />} />
          <Route path="/aboutme" element={<AboutMe />} />
        </Routes>
      </main>
    </div>
  );
}

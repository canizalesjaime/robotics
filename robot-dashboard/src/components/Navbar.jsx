import React from "react";
import { NavLink } from "react-router-dom";

const links = [
  { to: "/", label: "Motion Control" },
  { to: "/arm", label: "Arm Control" },
  { to: "/sensors", label: "Sensors Data" },
  { to: "/aboutme", label: "About Me" },
];

export default function Navbar() {
  return (
    <nav className="fixed top-0 left-0 right-0 bg-white shadow-md border-b border-gray-200 z-50">
      <div className="max-w-7xl mx-auto px-6 py-4 flex space-x-8">
        {links.map(({ to, label }) => (
          <NavLink
            key={to}
            to={to}
            className={({ isActive }) =>
              `text-gray-700 font-medium hover:text-blue-600 transition ${
                isActive ? "text-blue-600 border-b-2 border-blue-600" : ""
              }`
            }
            end
          >
            {label}
          </NavLink>
        ))}
      </div>
    </nav>
  );
}

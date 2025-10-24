import "./css/bootstrap_sketch.css";
import Navbar from "./Component/Navbar";
import { Routes, Route } from "react-router";
import { Container } from "react-bootstrap";
import Home from "./Component/Home";
import About from "./Component/About";
import Connection from "./Component/Connection";
import ROSLIB from "roslib";
import { useState, useEffect } from "react";

function App() {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros();
    rosInstance.on("connection", () => {
      console.log("✅ ROS connected");
      setConnected(true);
    });
    rosInstance.on("close", () => {
      console.log("❌ ROS disconnected");
      setConnected(false);
    });
    rosInstance.on("error", (err) => {
      console.error("ROS error:", err);
    });

    setRos(rosInstance);
  }, []);

  return (
    <Container fluid className="py-4">
      <Navbar />
      {ros && <Connection ros={ros} setConnected={setConnected} />}
      <Routes>
        <Route path="/" element={<Home ros={ros} connected={connected} />} />
        <Route path="/about" element={<About />} />
      </Routes>
    </Container>
  );
}

export default App;

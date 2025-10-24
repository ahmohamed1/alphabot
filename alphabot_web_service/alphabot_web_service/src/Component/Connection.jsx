import { useState } from "react";
import {Form, Button, Alert} from 'react-bootstrap';


function Connection({ ros, setConnected }) {
  const [urlForm, setUrlForm] = useState({ url: "" });
  const [connectionState, setConnectionState] = useState(false);

  const handleSubmit = () => {
    ros.on("connection", () => {
      console.log("Connection established!");
      setConnectionState(true);
      setConnected(true); // ✅ tell App
    });

    ros.on("close", () => {
      console.log("Connection closed!");
      setConnectionState(false);
      setConnected(false); // ✅ tell App
    });

    try {
      ros.connect(urlForm.url);
    } catch (error) {
      console.log(error);
    }
  };

  const handleChange = (e) => {
    const { name, value } = e.target;
    setUrlForm((prevForm) => ({
      ...prevForm,
      [name]: value,
    }));
  };

  return (
    <>
      <div className="d-flex align-items-center flex-nowrap mb-3">
        <Form.Group className="mb-3 d-flex align-items-center">
          <Form.Label htmlFor="url" className="mb-0 me-2 flex-shrink-0">
            Enter Robot URL
          </Form.Label>
          <Form.Control
            type="text"
            name="url"
            id="url"
            value={urlForm.url}
            onChange={handleChange}
            placeholder="ws://localhost:9090"
            className="me-3"
          />
          <Button variant="primary" onClick={handleSubmit}>
            Connect
          </Button>
        </Form.Group>
        <Alert
          className="text-center m-3"
          variant={connectionState ? "success" : "danger"}
        >
          {connectionState ? "Robot Connected" : "Robot Disconnect"}
        </Alert>
      </div>
    </>
  );
}

export default Connection;

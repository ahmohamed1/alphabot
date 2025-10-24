import TeleOperation from "./TeleOperation";
import { Row, Col } from "react-bootstrap";
import RobotState from "./RobotState";
import ImuData from "./ImuData";
import MapView from "./MapView";


function Home({ ros, connected }) {
  return (
    <>
      <Row>
        <h1>Welcome, to alphabot web</h1>
      </Row>
      <Row>
        <Col>
          <Row>
            <TeleOperation ros={ros} />
          </Row>
          <Row>
            <RobotState ros={ros}/>
          </Row>
        </Col>
        <Col>
          {connected ? <MapView ros={ros} /> : <p>Waiting for ROS connection...</p>}
        </Col>
      </Row>
      {/* <ImuData ros={ros} /> */}
    </>
  );
}

export default Home;

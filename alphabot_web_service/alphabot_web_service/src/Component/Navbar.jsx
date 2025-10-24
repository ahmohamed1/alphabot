import {
  Navbar as NavbarBS,
  Nav,
  Container,
} from "react-bootstrap";
import { Link, NavLink } from "react-router";

function Navbar (){

    return (
        <NavbarBS sticky="top" className={"shadow-lg mb-3"}>
            <Container>
                <Nav className="me-auto">
                    <Nav.Link to="/" as={NavLink}>Home</Nav.Link>
                    <Nav.Link to="/about" as={NavLink}>About</Nav.Link>
                </Nav>
            </Container>

        </NavbarBS>
    )
};

export default Navbar;
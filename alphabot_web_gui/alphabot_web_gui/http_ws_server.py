"""Zero-dependency AsyncIO HTTP 1.1 + RFC 6455 WebSocket server for ROS 2 Web GUI."""

import asyncio
import base64
import hashlib
import json
import mimetypes
import struct
import traceback
from pathlib import Path
from typing import Any, Callable, Coroutine, Dict, List, Optional, Set, Tuple
from urllib.parse import unquote, urlparse

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class WebSocketConnection:
    """Represents a connected RFC 6455 WebSocket client."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self.reader = reader
        self.writer = writer
        self.closed = False
        self._write_lock = asyncio.Lock()

    async def send_json(self, data: Any) -> None:
        text = json.dumps(data, separators=(",", ":"))
        await self.send_text(text)

    async def send_text(self, text: str) -> None:
        if self.closed:
            return
        payload = text.encode("utf-8")
        await self._send_frame(0x1, payload)

    async def _send_frame(self, opcode: int, payload: bytes) -> None:
        if self.closed:
            return
        header = bytearray()
        header.append(0x80 | (opcode & 0x0F))
        length = len(payload)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(127)
            header.extend(struct.pack("!Q", length))

        async with self._write_lock:
            try:
                self.writer.write(bytes(header) + payload)
                await self.writer.drain()
            except Exception:
                self.closed = True

    async def recv_message(self) -> Optional[str]:
        """Read next complete WebSocket text frame. Returns None on close/EOF."""
        fragments: List[bytes] = []
        while not self.closed:
            try:
                first_two = await self.reader.readexactly(2)
            except (asyncio.IncompleteReadError, ConnectionError, OSError):
                self.closed = True
                return None

            b1, b2 = first_two[0], first_two[1]
            fin = bool(b1 & 0x80)
            opcode = b1 & 0x0F
            masked = bool(b2 & 0x80)
            length = b2 & 0x7F

            try:
                if length == 126:
                    ext = await self.reader.readexactly(2)
                    length = struct.unpack("!H", ext)[0]
                elif length == 127:
                    ext = await self.reader.readexactly(8)
                    length = struct.unpack("!Q", ext)[0]

                mask_key = await self.reader.readexactly(4) if masked else b""
                payload = await self.reader.readexactly(length) if length > 0 else b""
            except (asyncio.IncompleteReadError, ConnectionError, OSError):
                self.closed = True
                return None

            if masked and payload:
                unmasked = bytearray(length)
                for i in range(length):
                    unmasked[i] = payload[i] ^ mask_key[i & 3]
                payload = bytes(unmasked)

            if opcode == 0x8:  # Close frame
                await self._send_frame(0x8, b"")
                self.closed = True
                return None
            elif opcode == 0x9:  # Ping -> Pong
                await self._send_frame(0xA, payload)
                continue
            elif opcode == 0xA:  # Pong
                continue
            elif opcode in (0x0, 0x1, 0x2):
                fragments.append(payload)
                if fin:
                    raw = b"".join(fragments)
                    return raw.decode("utf-8", errors="replace")

        return None

    async def close(self) -> None:
        self.closed = True
        try:
            self.reader.feed_eof()
        except Exception:
            pass
        try:
            self.writer.close()
        except Exception:
            pass


class AsyncHttpWsServer:
    """Lightweight AsyncIO HTTP + WebSocket Server for serving the GUI and API."""

    def __init__(self, static_dir: Path, host: str = "0.0.0.0", port: int = 8090):
        self.static_dir = Path(static_dir)
        self.host = host
        self.port = port
        self.routes: List[Tuple[str, str, Callable]] = []
        self.ws_handler: Optional[Callable[[WebSocketConnection], Coroutine]] = None
        self.clients: Set[WebSocketConnection] = set()
        self._client_tasks: Set[asyncio.Task] = set()
        self._server: Optional[asyncio.AbstractServer] = None

    def route(self, method: str, pattern: str):
        def decorator(func: Callable):
            self.routes.append((method.upper(), pattern, func))
            return func
        return decorator

    def websocket(self, func: Callable[[WebSocketConnection], Coroutine]):
        self.ws_handler = func
        return func

    async def start(self) -> None:
        self._server = await asyncio.start_server(
            self._track_client, self.host, self.port, reuse_address=True
        )

    async def _track_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        task = asyncio.current_task()
        if task is not None:
            self._client_tasks.add(task)
        try:
            await self._handle_client(reader, writer)
        except asyncio.CancelledError:
            pass
        finally:
            if task is not None:
                self._client_tasks.discard(task)

    async def stop(self) -> None:
        for client in list(self.clients):
            await client.close()
        self.clients.clear()
        for task in list(self._client_tasks):
            task.cancel()
        if self._client_tasks:
            await asyncio.gather(*self._client_tasks, return_exceptions=True)
        self._client_tasks.clear()
        if self._server is not None:
            self._server.close()
            try:
                await asyncio.wait_for(self._server.wait_closed(), timeout=0.5)
            except Exception:
                pass

    async def broadcast_json(self, data: Any) -> None:
        if not self.clients:
            return
        text = json.dumps(data, separators=(",", ":"))
        dead: List[WebSocketConnection] = []
        for client in list(self.clients):
            if client.closed:
                dead.append(client)
                continue
            try:
                await client.send_text(text)
            except Exception:
                dead.append(client)
        for d in dead:
            self.clients.discard(d)

    def _match_route(
        self, method: str, path: str
    ) -> Tuple[Optional[Callable], Dict[str, str]]:
        req_parts = [p for p in path.strip("/").split("/") if p]
        for r_method, r_pattern, handler in self.routes:
            if r_method != method:
                continue
            pat_parts = [p for p in r_pattern.strip("/").split("/") if p]
            if len(req_parts) != len(pat_parts):
                continue
            params: Dict[str, str] = {}
            matched = True
            for rp, pp in zip(req_parts, pat_parts):
                if pp.startswith("{") and pp.endswith("}"):
                    params[pp[1:-1]] = unquote(rp)
                elif rp != pp:
                    matched = False
                    break
            if matched:
                return handler, params
        return None, {}

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        try:
            header_bytes = await asyncio.wait_for(
                reader.readuntil(b"\r\n\r\n"), timeout=10.0
            )
        except Exception:
            writer.close()
            return

        header_text = header_bytes.decode("latin-1", errors="replace")
        lines = header_text.split("\r\n")
        if not lines or len(lines[0].split()) < 2:
            writer.close()
            return

        parts = lines[0].split()
        method = parts[0].upper()
        raw_target = parts[1]
        parsed_url = urlparse(raw_target)
        path = parsed_url.path

        headers: Dict[str, str] = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()

        # Check for WebSocket upgrade on /ws
        if (
            path == "/ws"
            and headers.get("upgrade", "").lower() == "websocket"
            and "sec-websocket-key" in headers
        ):
            ws_key = headers["sec-websocket-key"]
            accept_val = base64.b64encode(
                hashlib.sha1((ws_key + WS_GUID).encode("ascii")).digest()
            ).decode("ascii")
            handshake = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_val}\r\n\r\n"
            )
            writer.write(handshake.encode("ascii"))
            await writer.drain()

            ws_conn = WebSocketConnection(reader, writer)
            self.clients.add(ws_conn)
            try:
                if self.ws_handler:
                    await self.ws_handler(ws_conn)
            finally:
                self.clients.discard(ws_conn)
                await ws_conn.close()
            return

        # Handle CORS preflight
        if method == "OPTIONS":
            await self._send_response(writer, 204, b"", "text/plain")
            return

        # Read HTTP request body if present
        content_length = int(headers.get("content-length", "0"))
        body_bytes = b""
        if content_length > 0:
            try:
                body_bytes = await asyncio.wait_for(
                    reader.readexactly(content_length), timeout=10.0
                )
            except Exception:
                await self._send_json(writer, 400, {"detail": "Incomplete request body"})
                return

        # Match API route
        handler, path_params = self._match_route(method, path)
        if handler is not None:
            try:
                body_json = (
                    json.loads(body_bytes.decode("utf-8")) if body_bytes else {}
                )
            except Exception:
                await self._send_json(writer, 400, {"detail": "Invalid JSON body"})
                return

            try:
                result = await handler(body_json, path_params)
                if isinstance(result, tuple) and len(result) == 2:
                    status_code, payload = result
                else:
                    status_code, payload = 200, result
                await self._send_json(writer, status_code, payload)
            except ValueError as exc:
                await self._send_json(writer, 400, {"detail": str(exc)})
            except FileNotFoundError as exc:
                await self._send_json(writer, 404, {"detail": str(exc)})
            except Exception as exc:
                traceback.print_exc()
                await self._send_json(writer, 500, {"detail": str(exc)})
            return

        # Serve static files from web/
        if method == "GET":
            rel = "index.html" if path in ("/", "") else path.lstrip("/")
            target_file = (self.static_dir / rel).resolve()
            if (
                str(target_file).startswith(str(self.static_dir.resolve()))
                and target_file.is_file()
            ):
                mime, _ = mimetypes.guess_type(str(target_file))
                content = target_file.read_bytes()
                await self._send_response(
                    writer, 200, content, mime or "application/octet-stream"
                )
                return

        await self._send_json(writer, 404, {"detail": f"Not found: {path}"})

    async def _send_json(
        self, writer: asyncio.StreamWriter, status: int, data: Any
    ) -> None:
        raw = json.dumps(data).encode("utf-8")
        await self._send_response(writer, status, raw, "application/json; charset=utf-8")

    async def _send_response(
        self,
        writer: asyncio.StreamWriter,
        status: int,
        body: bytes,
        content_type: str,
    ) -> None:
        reasons = {
            200: "OK",
            201: "Created",
            204: "No Content",
            400: "Bad Request",
            404: "Not Found",
            500: "Internal Server Error",
        }
        reason = reasons.get(status, "OK")
        hdr = (
            f"HTTP/1.1 {status} {reason}\r\n"
            f"Content-Type: {content_type}\r\n"
            f"Content-Length: {len(body)}\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Access-Control-Allow-Methods: GET, POST, DELETE, OPTIONS\r\n"
            "Access-Control-Allow-Headers: Content-Type\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: close\r\n\r\n"
        )
        try:
            writer.write(hdr.encode("latin-1") + body)
            await writer.drain()
        except Exception:
            pass
        finally:
            try:
                writer.close()
            except Exception:
                pass

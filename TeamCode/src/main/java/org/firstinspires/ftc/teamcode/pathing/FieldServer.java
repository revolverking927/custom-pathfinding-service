package org.firstinspires.ftc.teamcode.pathing;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

/**
 * Serves the latest state JSON at GET /state for PathViewer/viewer.html. Plain sockets, no
 * dependencies. Always call {@link #stop()} (try/finally) so the port is free for the next run.
 */
public class FieldServer {
    private volatile String state = "{}";
    private volatile boolean running;
    private ServerSocket server;
    private Thread thread;

    public void start(int port) throws IOException {
        server = new ServerSocket();
        server.setReuseAddress(true);
        server.bind(new InetSocketAddress(port));
        running = true;
        thread = new Thread(this::acceptLoop, "FieldServer");
        thread.setDaemon(true);
        thread.start();
    }

    public void publish(String json) {
        state = json;
    }

    public void stop() {
        running = false;
        try {
            if (server != null) server.close();
        } catch (IOException ignored) {
            // Closing anyway.
        }
        if (thread != null) thread.interrupt();
    }

    private void acceptLoop() {
        while (running) {
            try (Socket socket = server.accept()) {
                socket.setSoTimeout(1000);
                handle(socket);
            } catch (IOException e) {
                // Server closed, or a client misbehaved: keep serving unless we were stopped.
            }
        }
    }

    private void handle(Socket socket) throws IOException {
        BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.US_ASCII));
        String requestLine = in.readLine();
        if (requestLine == null) return;
        // Drain headers.
        String line;
        while ((line = in.readLine()) != null && !line.isEmpty()) { /* skip */ }

        String status;
        String body;
        if (requestLine.startsWith("OPTIONS")) {
            status = "204 No Content";
            body = "";
        } else if (requestLine.startsWith("GET /state")) {
            status = "200 OK";
            body = state;
        } else {
            status = "404 Not Found";
            body = "{}";
        }
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        String head = "HTTP/1.1 " + status + "\r\n"
                + "Content-Type: application/json\r\n"
                + "Access-Control-Allow-Origin: *\r\n"
                + "Access-Control-Allow-Headers: *\r\n"
                + "Cache-Control: no-store\r\n"
                + "Content-Length: " + bytes.length + "\r\n"
                + "Connection: close\r\n\r\n";
        OutputStream out = socket.getOutputStream();
        out.write(head.getBytes(StandardCharsets.US_ASCII));
        out.write(bytes);
        out.flush();
    }
}

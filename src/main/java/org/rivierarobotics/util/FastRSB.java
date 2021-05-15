/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.util;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Faster method of transferring data to client(s) than NetworkTables. Uses a
 * 20ms loop (RoboRio minimum) instead of 100ms+ of NetworkTables
 * (Shuffleboard). UDP broadcast transport within FRC-legal ports.
 */
public class FastRSB {
    private static FastRSB INSTANCE = null;
    private static final int PORT = 5808;
    private final Map<String, Object> data;
    private DatagramSocket socket;
    private ScheduledFuture<?> thread;
    private InetAddress broadcastIp;

    private FastRSB() {
        this.data = new LinkedHashMap<>();
        this.thread = null;
        try {
            this.broadcastIp = getRioBroadcastAddress();
            this.socket = new DatagramSocket();
        } catch (SocketException | UnknownHostException e) {
            e.printStackTrace();
        }
    }

    /**
     * Get broadcast address of RoboRio automatically. Will get the address
     * starting with the 10.TE.AM.XXX prefix (from a matching interface,
     * which should be ethernet). Avoids localhost and loopback addresses.
     * Guesses if no broadcast address is found.
     *
     * @return the broadcast address of the RoboRio.
     * @throws SocketException if an interface is invalid.
     * @throws UnknownHostException if an interface cannot be routed.
     */
    private static InetAddress getRioBroadcastAddress() throws SocketException, UnknownHostException {
        Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
        while (interfaces.hasMoreElements()) {
            NetworkInterface networkInterface = interfaces.nextElement();
            if (!networkInterface.isLoopback() && networkInterface.isUp()) {
                for (InterfaceAddress addr : networkInterface.getInterfaceAddresses()) {
                    InetAddress bcast = addr.getBroadcast();
                    if (bcast != null && bcast.getHostAddress().startsWith("10.")) {
                        return bcast;
                    }
                }
            }
        }
        // Guess at broadcast address if interfaces are not configured correctly
        String curr = InetAddress.getLocalHost().getHostAddress();
        return InetAddress.getByName(curr.substring(0, curr.lastIndexOf('.') + 1) + "255");
    }

    public Object getEntry(String key) {
        return data.get(key);
    }

    public FastRSB setEntry(String key, Object value) {
        data.put(key, value);
        return this;
    }

    public FastRSB removeEntry(String key) {
        data.remove(key);
        return this;
    }

    public FastRSB clearEntries() {
        data.clear();
        return this;
    }

    /**
     * Encodes data to be sent from socket. Works on a key-value map system.
     * A 0x01 value is placed before a key and 0x02 before a value. All
     * values must be ASCII and single-byte within String objects.
     *
     * @return the encoded data in bytes to send to the socket.
     */
    private List<Byte> encodeData() {
        List<Byte> encData = new LinkedList<>();
        for (Map.Entry<String, Object> entry : data.entrySet()) {
            encData.add((byte) 0x01);
            for (char kc : entry.getKey().toCharArray()) {
                encData.add((byte) kc);
            }
            encData.add((byte) 0x02);
            // Uses toString() method of value objects for data
            for (char vc : entry.getValue().toString().toCharArray()) {
                encData.add((byte) vc);
            }
        }
        return encData;
    }

    /**
     * Send encoded data to the client(s).
     */
    private void send() {
        List<Byte> encData = encodeData();
        byte[] sendData = new byte[encData.size()];
        for (int i = 0; i < sendData.length; i++) {
            sendData[i] = encData.get(i);
        }
        try {
            DatagramPacket packet = new DatagramPacket(sendData, sendData.length, broadcastIp, PORT);
            socket.send(packet);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean isStarted() {
        return thread == null;
    }

    public void start() {
        start(20); // Match RIO loop time
    }

    /**
     * Starts sending packets every msPeriod with a thread executor. Does not
     * use normal threads or clock implementation for efficiency.
     *
     * @param msPeriod milliseconds between data sending.
     */
    public void start(int msPeriod) {
        if (!isStarted()) {
            ScheduledExecutorService execSvc = Executors.newSingleThreadScheduledExecutor();
            thread = execSvc.scheduleAtFixedRate(this::send, 0, msPeriod, TimeUnit.MILLISECONDS);
        }
    }

    public void stop() {
        if (isStarted()) {
            thread.cancel(true);
            thread = null;
        }
    }

    public static FastRSB getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new FastRSB();
        }
        return INSTANCE;
    }
}


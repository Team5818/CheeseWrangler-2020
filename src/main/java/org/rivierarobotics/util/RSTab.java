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

import edu.wpi.cscore.VideoException;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Represents a <code>Shuffleboard</code> tab as part of the <code>RobotShuffleboard</code>.
 *
 * <p>Contains WPILib API tab object and internal lists for tiles.
 * Can add primitives, <code>Sendable</code>s, <code>CameraServer</code> streams,
 * and <code>VideoSources</code>. Tables can also be created in tabs.
 * Positioning and sizing of each tile is managed by <code>RSTileOptions</code>.</p>
 *
 * <p>Methods are arranged as a builder. Daisy-chaining is encouraged.</p>
 *
 * @see RobotShuffleboard
 * @see RSTileOptions
 * @see RSTable
 */
public class RSTab {
    private final ShuffleboardTab tab;
    private final String name;
    private final Map<String, NetworkTableEntry> entries;
    private final Map<String, RSTable> tables;

    /**
     * Creates a new tab with a specified name.
     *
     * <p>If an existing tab exists on the <code>Shuffleboard</code> with the same name,
     * any <code>SimpleWidget</code> tiles will be logged as entries. Existing
     * tables will not be logged and may either be added again or re-initialized.</p>
     *
     * @param name the title/name of the <code>Shuffleboard</code> tab to create.
     */
    public RSTab(String name) {
        this.tab = Shuffleboard.getTab(name);
        this.name = name;
        this.entries = new LinkedHashMap<>();
        this.tables = new LinkedHashMap<>();

        for (ShuffleboardComponent<?> comp : tab.getComponents()) {
            if (comp instanceof SimpleWidget) {
                entries.put(comp.getTitle(), ((SimpleWidget) comp).getEntry());
            }
        }
    }

    /**
     * Adds a named <code>CameraServer</code> stream to the <code>RSTab</code>.
     * Uses default {@link RSTileOptions}.
     *
     * <p>Wrapper for {@link #setCamera(String, RSTileOptions)}.</p>
     *
     * @see #setCamera(String, RSTileOptions)
     */
    public RSTab setCamera(String name) throws VideoException {
        return setCamera(name, RSTileOptions.DEFAULT);
    }

    /**
     * Adds a named <code>CameraServer</code> stream to the <code>RSTab</code> with properties.
     * Grabs stream source from the <code>CameraServer</code>.
     *
     * <p>Wrapper for {@link #setVideoSource(VideoSource, RSTileOptions)}.</p>
     *
     * @param name the named <code>CameraServer</code> to add.
     * @param options camera placement and sizing options on the <code>RSTab</code>.
     * @return the current <code>RSTab</code> where the <code>CameraServer</code> stream was added.
     * @throws VideoException if a video error occurred while retrieving the <code>CameraServer</code> stream.
     *
     * @see #setVideoSource(VideoSource, RSTileOptions)
     */
    public RSTab setCamera(String name, RSTileOptions options) throws VideoException {
        return setVideoSource(CameraServer.getInstance().getVideo(name).getSource(), options);
    }

    /**
     * Adds a <code>VideoSource</code> to the <code>RSTab</code>.
     * Uses default {@link RSTileOptions}.
     *
     * <p>Wrapper for {@link #setVideoSource(VideoSource, RSTileOptions)}.</p>
     *
     * @param src the <code>VideoSource</code> to add.
     * @return the current <code>RSTab</code> where the <code>VideoSource</code> stream was added.
     *
     * @see #setVideoSource(VideoSource, RSTileOptions)
     */
    public RSTab setVideoSource(VideoSource src) {
        return setVideoSource(src, RSTileOptions.DEFAULT);
    }

    /**
     * Adds a <code>VideoSource</code> to the <code>RSTab</code> with properties.
     *
     * <p>Applied as a <code>ComplexWidget</code> type to the WPILib <code>ShuffleboardTab</code>.<br>
     * Title/name of <code>Shuffleboard</code> tile equal to source name of <code>VideoSource</code>.</p>
     *
     * @param src the video source to add.
     * @param options video source placement and sizing options on the <code>RSTab</code>.
     * @return the current <code>RSTab</code> where the <code>VideoSource</code> stream was added.
     */
    public RSTab setVideoSource(VideoSource src, RSTileOptions options) {
        options.applyToComplex(tab.add(src.getName(), src));
        return this;
    }

    /**
     * Adds a <code>Sendable</code> type object to the <code>RSTab</code>.
     * Uses default {@link RSTileOptions}.
     *
     * <p>Wrapper for {@link #setSendable(Sendable, RSTileOptions)}.</p>
     *
     * @see #setSendable(Sendable, RSTileOptions)
     */
    public RSTab setSendable(Sendable sendable) {
        return setSendable(sendable, RSTileOptions.DEFAULT);
    }

    /**
     * Adds a <code>Sendable</code> type object to the <code>RSTab</code> with properties.
     *
     * <p>Applied as a <code>ComplexWidget</code> type to the WPILib <code>ShuffleboardTab</code>.<br>
     * Title/name of <code>Shuffleboard</code> tile equal to registered name of <code>Sendable</code>.<br>
     * Typically used for WPILib <code>SendableChooser</code> objects.</p>
     *
     * @param sendable the sendable type object to add.
     * @param options placement and sizing options on the <code>RSTab</code>.
     * @return the current <code>RSTab</code> where the <code>Sendable</code> was added.
     */
    public RSTab setSendable(Sendable sendable, RSTileOptions options) {
        options.applyToComplex(tab.add(SendableRegistry.getName(sendable), sendable));
        return this;
    }

    /**
     * Adds a primitive value to the <code>RSTab</code>.
     * Uses default {@link RSTileOptions}.
     *
     * <p>Wrapper for {@link #setEntry(String, Object, RSTileOptions)}.</p>
     *
     * @see #setEntry(String, Object, RSTileOptions)
     */
    public <T> RSTab setEntry(String title, T value) {
        return setEntry(title, value, RSTileOptions.DEFAULT);
    }

    /**
     * Adds a primitive value to the <code>RSTab</code> with properties.
     *
     * <p>Ensures a key with the same title does not already exist in logged entries, then
     * creates a new <code>SimpleWidget</code> with matching default value type.
     * If not, the entry is retrieved from the local listing and the value is set.<br>
     * Note that the tile's type will be forced to the newest value.<br>
     * It is suggested to use primitive types only, as only a limited subset of objects
     * are compatible. Most are wrapped by other methods in <code>RSTab</code>.</p>
     *
     * @param title the key/title of the <code>Shuffleboard</code> tile.
     * @param value the primitive value to set.
     * @param options placement and sizing options on the <code>RSTab</code>.
     * @return the current <code>RSTab</code> where the value was added.
     */
    public <T> RSTab setEntry(String title, T value, RSTileOptions options) {
        if (!entries.containsKey(title)) {
            entries.put(title, options.applyToSimple(tab.add(title, value)).getEntry());
        } else {
            entries.get(title).forceSetValue(value);
        }
        return this;
    }

    /**
     * Retrieves the <code>NetworkTableEntry</code> associated with a named tile.
     *
     * <p>Entries will be taken from the pre-existing entries list. If there is no
     * entry of a matching title in the list, one will be created with a default
     * value of an empty String. Note that the value must be force-set if done
     * manually if is is not a String.</p>
     *
     * @param title the name/key of the <code>Shuffleboard</code> tile.
     * @return the logged entry associated with the name/key.
     */
    public NetworkTableEntry getEntry(String title) {
        NetworkTableEntry entry = entries.get(title);
        if (entry == null) {
            entry = tab.add(title, "").getEntry();
            entries.put(title, entry);
        }
        return entry;
    }

    public Map<String, NetworkTableEntry> getEntries() {
        return entries;
    }

    /**
     * Retrieves a key/value table from the <code>RSTab</code>.
     * Uses default {@link RSTileOptions}.
     *
     * <p>Wrapper for {@link #getTable(String, RSTileOptions)}.</p>
     *
     * @see #getTable(String, RSTileOptions)
     */
    public RSTable getTable(String tableName) {
        return getTable(tableName, RSTileOptions.DEFAULT);
    }

    /**
     * Retrieves a key/value table from the <code>RSTab</code> with properties.
     *
     * <p>If the table does not exist (or is not present in the logged tables list)
     * a new one will be created. Note that height for tables has a maximum of 3 units.
     * Properties will not be used for preexisting tables.<br>
     * This is the suggested method of managing table access.</p>
     *
     * @param tableName the name/key/title of the table to get.
     * @param options placement and sizing options on the <code>RSTab</code>.
     * @return the resulting table for adding key/value pairs to.
     */
    public RSTable getTable(String tableName, RSTileOptions options) {
        if (tables.get(tableName) == null) {
            addTable(tableName, options);
        }
        return tables.get(tableName);
    }

    /**
     * Adds a new key/value table to the <code>RSTab</code>.
     * Uses default {@link RSTileOptions}.
     *
     * <p>Wrapper for {@link #addTable(String, RSTileOptions)}.</p>
     *
     * @see #addTable(String, RSTileOptions)
     */
    public void addTable(String tableName) {
        addTable(tableName, RSTileOptions.DEFAULT);
    }

    /**
     * Adds a new key/value table to the <code>RSTab</code> with properties.
     *
     * <p>Creates a new table and then places it on current tab.</p>
     *
     * @param tableName the name/key/title of the table to add.
     * @param options placement and sizing options on the <code>RSTab</code>.
     */
    public void addTable(String tableName, RSTileOptions options) {
        tables.put(tableName, new RSTable(tableName, this, options));
    }

    /**
     * Adds an existing key/value table to the <code>RSTab</code>.
     *
     * <p>Does not make a new table. The pre-exsting table is instead passed through and added.
     * The table name will be preset by the <code>ShuffleboardTable</code> object.</p>
     *
     * @param table the table object to display on the current tab.
     *
     * @see #addTable(String, RSTileOptions)
     */
    public void addTable(RSTable table) {
        tables.put(table.getName(), table);
    }

    public String getName() {
        return name;
    }

    public ShuffleboardTab getAPITab() {
        return tab;
    }
}

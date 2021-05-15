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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Represents a {@code Shuffleboard}-based table displayed on a {@link RSTab}.
 *
 * <p>Uses {@code BuiltInLayouts.kList} to create a new list (table) layout on a tab.
 * Entries are then created using {@code withWidget("NetworkTableTree")}
 * or retrieved from the internal entries list.
 * Positioning and sizing of the table is managed by {@link RSTileOptions}.</p>
 *
 * <p>Methods are arranged as a builder. Daisy-chaining is encouraged.</p>
 *
 * @see RSTab
 * @see RSTileOptions
 */
public class RSTable {
    private final LinkedHashMap<String, NetworkTableEntry> entries;
    private final String tableName;
    private final ShuffleboardLayout layout;
    private final RSTileOptions options;

    /**
     * Constructs a new table on a given {@link RSTab}.
     *
     * <p>Uses the {@code BuiltInLayouts.kList} enum to signal a table.
     * (The name indicates a list but is actually a key/value list).
     * The table label is hidden by default (clutters UI).
     * Does not automatically add the tab's contents to the table, for
     * that see {@link #addTabData(RSTab)}.</p>
     *
     * @param tableName the name/title of the table to be created.
     * @param tab the tab to place the new table onto.
     * @param options placement and sizing options for the {@link RSTable}.
     */
    public RSTable(String tableName, RSTab tab, RSTileOptions options) {
        this.tableName = tableName;
        this.options = options;
        this.layout = tab.getAPITab()
            .getLayout(tableName, BuiltInLayouts.kList)
            .withSize(options.getWidth(), options.getHeight())
            .withPosition(options.getPosX(), options.getPosY())
            .withProperties(Map.of("Label position", "HIDDEN"));
        this.entries = new LinkedHashMap<>();
    }

    /**
     * Adds a single key/title and value to the current table.
     *
     * <p>First checks for a matching key is in the entry log for this table.
     * If present, the entry's value will be updated to reflect the value passed.
     * If not present, a new entry will be created by adding it to the table
     * layout created in the constructor.
     * Note that {@code withWidget("NetworkTableTree")} is called to ensure
     * the key/value appears within the table and not as a separate tile.</p>
     *
     * @param name the name of the key/value pair to add.
     * @param value the value of the key/value pair to add.
     * @return the current table with entries added.
     */
    public RSTable setEntry(String name, Object value) {
        if (entries.containsKey(name)) {
            entries.get(name).forceSetValue(value);
        } else {
            entries.put(name, layout.add(tableName + "/" + name, value)
                .withWidget("Network Table Tree")
                .getEntry());
        }
        return this;
    }

    /**
     * Adds all key/value tiles from a passed tab to the current table.
     *
     * <p>Wrapper for {@link #addEntries(Collection)}.</p>
     *
     * @param tab the tab to add to the current/target tab.
     * @return the target {@link RSTable} which the parameter tab was appended to.
     *
     * @see #addEntries(Collection)
     */
    public RSTable addTabData(RSTab tab) {
        return addEntries(tab.getEntries().values());
    }

    /**
     * Adds tiles from a list to the current table.
     *
     * <p>Calls {@link #setEntry(String, Object)} internally for each {@code NetworkTableEntry}.
     * Key/title names are based on the title of the {@code NetworkTableEntry}
     * and should be automatically assigned. Do not change these manually.</p>
     *
     * @param ntEntries the entries to add to the current table.
     * @return the current {@link RSTable} where the entries were added.
     *
     * @see #setEntry(String, Object)
     */
    public RSTable addEntries(Collection<NetworkTableEntry> ntEntries) {
        for (NetworkTableEntry entry : ntEntries) {
            setEntry(entry.getName().substring(13), entry.getValue().getValue());
        }
        return this;
    }

    /**
     * Copy the current table to a passed {@link RSTab}.
     *
     * <p>Creates a new table with the same table name, options, and entries,
     * just targeted towards a new tab. Does not point the current tab
     * to the passed tab.</p>
     *
     * @param targetTab the target tab to copy the current table onto.
     * @return the new {@link RSTable} where the entries were copied to.
     */
    public RSTable copyToTab(RSTab targetTab) {
        RSTable table = new RSTable(tableName, targetTab, options).addEntries(entries.values());
        targetTab.addTable(table);
        return table;
    }

    public String getName() {
        return tableName;
    }
}

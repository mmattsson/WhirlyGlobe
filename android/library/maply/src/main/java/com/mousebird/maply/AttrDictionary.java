/*
 *  AttrDictionary.java
 *  WhirlyGlobeLib
 *
 *  Created by Steve Gifford on 6/2/14.
 *  Copyright 2011-2014 mousebird consulting
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

package com.mousebird.maply;

/**
 * The attribute dictionary is a collection of name/value pairs.
 * We use it to store (and modify) attributes on VectorObject structures,
 * primarily.  It's analogous to the NSDictionary class on iOS, but much
 * simpler.
 *
 */
public class AttrDictionary
{	
	/**
	 * Construct an empty attribution dictionary
	 */
	public AttrDictionary()
	{
		initialise();
	}

	/**
	 * Parse from a JSON file.
	 * Returns false on failure.
	 */
	public native boolean parseFromJSON(String json);
	
	/**
	 * Return a string corresponding to the given attribute name.
	 * @param attrName Name of the attribute we're looking for.
	 * @return Returns a string for the attribute or null if there was none.
	 */
	public native String getString(String attrName);
	
	/**
	 * Return an Integer corresponding to the given attribute name.
	 * @param attrName Name of the attribute we're looking for.
	 * @return Returns an Integer for the given attribute or null if there was none.
	 */
	public native Integer getInt(String attrName);
	
	/**
	 * Fetch a double corresponding to the given attribute name.
	 * @param attrName Name of the attribute we're looking for.
	 * @return Returns a Double for the given attribute or null if there was none.
	 */
	public native Double getDouble(String attrName);

	/**
	 * Fetch an identity corresponding to the given name.
	 * @param attrName Name of the attribute we're looking for.
	 * @return Returns a 64 bit identity the system uses for cataloging assets.
	 */
	public native Long getIdentity(String attrName);

	/**
	 * Fetch an Object corresponding to the given attribute name.
	 * @param attrName Name of the attribute we're looking for.
	 * @return Returns an Object for the given attribute or null if there was none.
	 */
	public native Object get(String attrName);

	/**
	 * Fetch a dictionary corresponding to the given attribute name.
	 * @param attrName Name of the attribute we're looking for.
	 * @return Returns the dictionary for the attribute name or null if there was none.
	 */
	public native AttrDictionary getDict(String attrName);

	/**
	 * Return all the top level attribute names.
	 */
	public native String[] getKeys();

	/**
	 * Set a string value.
	 * @param attrName Name of the attribute to set.
	 * @param attrVal String value to set.
     */
	public native void setString(String attrName,String attrVal);

	/**
	 * Set an int value.
	 * @param attrName Name of the attribute to set.
	 * @param attrVal Int value to set.
     */
	public native void setInt(String attrName,int attrVal);

	/**
	 * Set a double value.
	 * @param attrName Name of the attribute to set.
	 * @param attrVal Double value to set.
     */
	public native void setDouble(String attrName,double attrVal);

	// Convert to a string for debugging
	public native String toString();

    // Merge in key-value pairs from another dictionary
    public native void addEntries(AttrDictionary other);
	
	static
	{
		nativeInit();
	}
	private static native void nativeInit();
	public void finalize()
	{
		dispose();
	}
	native void initialise();
	native void dispose();	
	private long nativeHandle;	
}

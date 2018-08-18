/**
 * Copyright 2018 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files (the "Software"), to deal in the Software without restriction, including without 
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following 
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions 
 * of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 * 
 */
package org.tahomarobotics.robot.util;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.dataformat.smile.SmileFactory;

public class PathData {
	
	private final List<double[]> paths;

	public PathData() {
		this(new ArrayList<>());
	}
	
	private PathData(List<double[]> paths) {
		this.paths = paths;
	}
	
	public void clearData() {
		paths.clear();
	}
	
	public void addPath(double[] points) {
		assert(points != null && (points.length % 2 == 0));
		paths.add(points);
	}
	
	public List<double[]> getPaths() {
		return paths;
	}
	
	public byte[] serialize() {
		ByteArrayOutputStream out = new ByteArrayOutputStream();
		ObjectMapper mapper = new ObjectMapper(new SmileFactory());
		ObjectNode node = mapper.createObjectNode();
		node.putPOJO("paths", paths);
		
		try {
			mapper.writeValue(out, node);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return out.toByteArray();
	}
	
	public static PathData deserialize(byte[] json) {
		try {
			ObjectMapper mapper = new ObjectMapper(new SmileFactory());
			JsonNode root = mapper.readTree(json);
			JsonNode dataNode = root.findValue("paths");
			if (!dataNode.isMissingNode() && dataNode.isArray()) {
				ObjectReader reader = mapper.readerFor(new TypeReference<List<double[]>>() { });
				return new PathData(reader.readValue(dataNode));
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		return null;
	}
}

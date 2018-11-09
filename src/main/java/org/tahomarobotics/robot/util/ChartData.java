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

public class ChartData {
	
	private final List<String> names = new ArrayList<>();
	private final List<double[]> data = new ArrayList<>();
	
	protected ChartData(List<String> names) {
		for (String name : names) {
			this.names.add(name);
		}
	}
	
	public ChartData(String title, String xaxis, String yaxis, String[] seriesNames) {
		names.add(title);
		names.add(xaxis);
		names.add(yaxis);
		for (String name : seriesNames) {
			names.add(name);
		}
	}
	
	public void addData(double[] data) {
		assert(data != null && data.length == (names.size() - 2));
		this.data.add(data);
	}
	
	public void clear() {
		data.clear();
	}
	
	public byte[] serialize() {
		
		ByteArrayOutputStream out = new ByteArrayOutputStream();
		ObjectMapper mapper = new ObjectMapper(new SmileFactory());
		ObjectNode node = mapper.createObjectNode();
		node.putPOJO("names", names);
		node.putPOJO("data", data);
		
		try {
			mapper.writeValue(out, node);
		} catch (IOException e) {
			e.printStackTrace();
		}

		return out.toByteArray();
	}
	
	public static ChartData deserialize(byte[] json) {
		
		try {
			ObjectMapper mapper = new ObjectMapper(new SmileFactory());
			JsonNode root = mapper.readTree(json);
			JsonNode namesNode = root.findValue("names");
			if (!namesNode.isMissingNode() && namesNode.isArray()) {
				ObjectReader reader = mapper.readerFor(new TypeReference<List<String>>() {
				});
				List<String> list = reader.readValue(namesNode);
				ChartData chartData = new ChartData(list);

				JsonNode dataNode = root.findValue("data");
				if (!dataNode.isMissingNode() && dataNode.isArray()) {
					reader = mapper.readerFor(new TypeReference<List<double[]>>() {
					});
					chartData.setData(reader.readValue(dataNode));
					return chartData;
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}

		return null;
	}
	
	public String getTitle() {
		return names.get(0);
	}
	public String getXAxis() {
		return names.get(1);
	}
	public String getYAxis() {
		return names.get(2);
	}
	public int getSeriesCount() {
		return names.size() - 3;
	}
	
	public String getSeriesName(int i) {
		return names.get(3+i);
	}
	
	public List<double[]> getData() {
		return data;
	}
	
	private void setData( List<double[]> data) {
		this.data.addAll(data);
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		
		sb.append("Title: ").append(getTitle()).append('\n');
		sb.append("X-Axis: ").append(getXAxis()).append('\n');
		sb.append("Y-Axis: ").append(getYAxis()).append('\n');
		sb.append("Title: ").append(getTitle()).append('\n');
		sb.append("x ");
		for(int c = 0; c < getSeriesCount(); c++) {
			sb.append(getSeriesName(c)).append(' ');
		}
		sb.append('\n');
		for(double[] r : getData()) {
			for(double d : r) {
				sb.append(d).append(' ');
			}
			sb.append('\n');
		}
		
		return sb.toString();
	}
	
	public static void main(String args[]) throws IOException {
		ChartData chartData = new ChartData("Lift Motion", "Time (sec)", "Velocity (deg/sec)", new String[] {"sin", "cos", "tan"});
		for (double x = 0.0; x < 10.0; x += 0.1) {
			double y = Math.cos(x);
			double z = Math.sin(x);
			double zz = Math.tan(x);
			chartData.addData(new double[] {x, y, z, zz});
		}
		
		for(int i = 0; i < 100; i++) {
		long start = System.nanoTime();
		byte[] raw = chartData.serialize();
		long s = System.nanoTime();
		chartData = ChartData.deserialize(raw);
		long d = System.nanoTime();
		
		
		System.out.println("serialize: " + 1e-6*(s - start));
		System.out.println("deserialize: " + 1e-6*(d - s));
		
		}
	}
}

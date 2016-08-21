package com.graphhopper.reader.dem;

import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import com.graphhopper.coll.GHBitSet;
import com.graphhopper.coll.GHBitSetImpl;
import com.graphhopper.routing.util.AllEdgesIterator;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.util.BreadthFirstSearch;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.PointList;

public class AbstractElevationInterpolator {

	private final GraphHopperStorage storage;
	private final List<FlagEncoder> flagEncoders;
	private final int key = FlagEncoder.K_TUNNEL;

	public AbstractElevationInterpolator(GraphHopperStorage storage) {
		this.storage = storage;
		this.flagEncoders = storage.getEncodingManager().fetchEdgeEncoders();
	}

	private boolean isMatching(long flags) {
		for (FlagEncoder encoder : flagEncoders) {
			if (encoder.isBool(flags, key)) {
				return true;
			}
		}
		return false;
	}

	public void execute() {
		final AllEdgesIterator allEdges = storage.getAllEdges();
		final int maxEdgesId = allEdges.getMaxId();
		final GHBitSet visitedEdges = new GHBitSetImpl(maxEdgesId);
		final EdgeExplorer edgeExplorer = storage.createEdgeExplorer();

		while (allEdges.next()) {
			final int edgeId = allEdges.getEdge();
			final long flags = allEdges.getFlags();
			if (isMatching(flags)) {
				if (!visitedEdges.contains(edgeId)) {
					processStructureEdges(allEdges, visitedEdges, edgeExplorer, edgeId);
				}
			}
			visitedEdges.add(edgeId);
		}

		final AllEdgesIterator allEdges1 = storage.getAllEdges();
		while (allEdges1.next()) {
			final long flags = allEdges1.getFlags();
			if (isMatching(flags)) {
				int firstNodeId = allEdges1.getBaseNode();
				int secondNodeId = allEdges1.getAdjNode();

				double lat0 = storage.getNodeAccess().getLat(firstNodeId);
				double lon0 = storage.getNodeAccess().getLon(firstNodeId);
				double ele0 = storage.getNodeAccess().getEle(firstNodeId);

				double lat1 = storage.getNodeAccess().getLat(secondNodeId);
				double lon1 = storage.getNodeAccess().getLon(secondNodeId);
				double ele1 = storage.getNodeAccess().getEle(secondNodeId);

				PointList pointList = allEdges1.fetchWayGeometry(0);
				final int count = pointList.size();
				for (int index = 0; index < count; index++) {
					double lat = pointList.getLat(index);
					double lon = pointList.getLon(index);
					double ele = pointList.getEle(index);
					double newEle = calculateElevation(lat, lon, lat0, lon0, ele0, lat1, lon1, ele1);
					pointList.set(index, lat, lon, newEle);
				}
				allEdges1.setWayGeometry(pointList);
			}
		}
	}

	private void processStructureEdges(final AllEdgesIterator edgesIterator, final GHBitSet visitedEdgesBitSet,
			final EdgeExplorer edgeExplorer, int edgeId) {
		int baseNode = edgesIterator.getBaseNode();
		final Set<Integer> innerNodes = new HashSet<>();
		final Set<Integer> outerNodes = new HashSet<>();
		new BreadthFirstSearch() {

			protected boolean checkAdjacent(EdgeIteratorState edge) {
				final Integer edgeId = edge.getEdge();
				visitedEdgesBitSet.add(edgeId.intValue());
				final Integer baseNode = edge.getBaseNode();
				boolean matching = isMatching(edge.getFlags());
				if (!matching) {
					innerNodes.remove(baseNode);
					outerNodes.add(baseNode);
				} else {
					if (!outerNodes.contains(baseNode)) {
						innerNodes.add(baseNode);
					}
				}
				return matching;
			}

		}.start(edgeExplorer, baseNode);
		recalculateElevationsOfInnerNodes(outerNodes, innerNodes);
	}

	private void recalculateElevationsOfInnerNodes(final Set<Integer> outerNodes, final Set<Integer> innerNodes) {
		if (outerNodes.size() == 0) {
			// do nothing
		} else if (outerNodes.size() == 1) {
			final Iterator<Integer> outerNodeIdsIterator = outerNodes.iterator();
			int firstNodeId = outerNodeIdsIterator.next();
			double ele0 = storage.getNodeAccess().getEle(firstNodeId);
			for (Integer innerNodeId : innerNodes) {
				double lat = storage.getNodeAccess().getLat(innerNodeId);
				double lon = storage.getNodeAccess().getLon(innerNodeId);
				double ele = storage.getNodeAccess().getEle(innerNodeId);
				double newEle = ele0;
				storage.getNodeAccess().setNode(innerNodeId, lat, lon, newEle);
				System.out.println("Elevation of inner node [" + innerNodeId + "] corrected from [" + ele + "] to ["
						+ newEle + "].");
			}
		} else if (outerNodes.size() == 2) {
			final Iterator<Integer> outerNodeIdsIterator = outerNodes.iterator();
			int firstNodeId = outerNodeIdsIterator.next();
			int secondNodeId = outerNodeIdsIterator.next();

			double lat0 = storage.getNodeAccess().getLat(firstNodeId);
			double lon0 = storage.getNodeAccess().getLon(firstNodeId);
			double ele0 = storage.getNodeAccess().getEle(firstNodeId);
			System.out.println("Elevation of the first outer node [" + firstNodeId + "] is [" + ele0 + "].");

			double lat1 = storage.getNodeAccess().getLat(secondNodeId);
			double lon1 = storage.getNodeAccess().getLon(secondNodeId);
			double ele1 = storage.getNodeAccess().getEle(secondNodeId);
			System.out.println("Elevation of the second outer node [" + secondNodeId + "] is [" + ele1 + "].");

			for (Integer innerNodeId : innerNodes) {
				double lat = storage.getNodeAccess().getLat(innerNodeId);
				double lon = storage.getNodeAccess().getLon(innerNodeId);
				double ele = storage.getNodeAccess().getEle(innerNodeId);
				double newEle = calculateElevation(lat, lon, lat0, lon0, ele0, lat1, lon1, ele1);
				storage.getNodeAccess().setNode(innerNodeId, lat, lon, newEle);
				System.out.println("Elevation of inner node [" + innerNodeId + "] corrected from [" + ele + "] to ["
						+ newEle + "].");
			}
		} else if (outerNodes.size() > 2) {
			final PointList pointList = new PointList(outerNodes.size(), true);
			final Iterator<Integer> outerNodeIdsIterator = outerNodes.iterator();
			while (outerNodeIdsIterator.hasNext()) {
				int outerNodeId = outerNodeIdsIterator.next();
				pointList.add(storage.getNodeAccess().getLat(outerNodeId),
						storage.getNodeAccess().getLon(outerNodeId), storage.getNodeAccess().getEle(outerNodeId));
			}
			for (Integer innerNodeId : innerNodes) {
				double lat = storage.getNodeAccess().getLat(innerNodeId);
				double lon = storage.getNodeAccess().getLon(innerNodeId);
				double ele = storage.getNodeAccess().getLon(innerNodeId);
				double newEle = calculateElevation(lat, lon, pointList);
				storage.getNodeAccess().setNode(innerNodeId, lat, lon, newEle);
				System.out.println("Elevation of inner node [" + innerNodeId + "] corrected from [" + ele + "] to ["
						+ newEle + "].");
			}
		}
	}

	private double calculateElevation(double lat, double lon, double lat0, double lon0, double ele0, double lat1,
			double lon1, double ele1) {
		double dlat0 = lat0 - lat;
		double dlon0 = lon0 - lon;
		double dlat1 = lat1 - lat;
		double dlon1 = lon1 - lon;
		double d0 = Math.sqrt(dlon0 * dlon0 + dlat0 * dlat0);
		double d1 = Math.sqrt(dlon1 * dlon1 + dlat1 * dlat1);
		double dsum = d0 + d1;

		double newEle = dsum < 0.00001 ? (d0 < d1 ? ele0 : ele1) : ele0 + (ele1 - ele0) * d0 / (dsum);
		return newEle;
	}

	private double calculateElevation(double lat, double lon, PointList pointList) {
		double[] d = new double[pointList.size()];
		double[] e = new double[pointList.size()];
		double sum = 0;
		for (int index = 0; index < pointList.size(); index++) {
			double lati = pointList.getLat(index);
			double loni = pointList.getLat(index);
			double dlat = lati - lat;
			double dlon = loni - lon;
			double dist = Math.sqrt(dlat * dlat + dlon * dlon);
			e[index] = pointList.getEle(index);
			if (dist < 0.00001) {
				return e[index];
			}
			d[index] = 1 / dist;
			sum += d[index];
		}

		double ele = 0;

		for (int index = 0; index < pointList.size(); index++) {
			double elei = pointList.getEle(index);
			ele += elei * d[index] / sum;
		}
		return ele;
	}

}

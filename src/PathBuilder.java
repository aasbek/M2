import java.util.PriorityQueue;
import java.util.Vector;

public class PathBuilder {
	public Vector<Node> nodes;
	public Vector<Node> pickupNodes;
	public Vector<Node> deliveryNodes;
	public Vector<Node> depot;
	public InstanceData inputdata;
	// public ArrayList<Hashtable<Integer, Boolean>> feasibility;
	private double zeroTol = 0.001;
	private int numberOfDominatedLabels;
	
	// public static ArrayList<Hashtable<Integer, Boolean>> feasibilityTest; 
	
	public PathBuilder(Vector<Node> pickupNodes, Vector<Node> deliveryNodes, Vector<Node> nodes, Vector<Node> depot, InstanceData inputdata) {
		this.pickupNodes = pickupNodes;
		this.nodes = nodes;
		this.deliveryNodes = deliveryNodes;
		this.depot = depot;
		this.inputdata = inputdata;
		// feasibilityTest = new ArrayList<Hashtable<Integer, Boolean>>();
		numberOfDominatedLabels = 0;
	}
	
	
	
	
	public Label LabelExtension(Node node, Label L) {
		
		// Returns null if the node is already visited
		if(L.path.contains(node.number)) {
			return null;
		}		
		
		// Returns null if the node is unreachable 
		if(L.unreachablePickupNodes.contains(node.number)) {
			return null;
		}
		
		// Cannot return to start depot
		if(node.number == 0){
			return null;
		}
		
		// Cannot leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		// Cannot arrive at end depot without delivering every pickup
		if(node.number == 1 && !L.openNodes.isEmpty()){
			return null;	
		}
		
		// Cannot visit a delivery node whose pickup node has not been visited 
		if (node.type == "DeliveryNode" && ! L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){	
			return null;
		}
	
		
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		
		L2.path = new Vector<Integer>();
		// Adding all elements from the predecessor's path to this label's path
		for(int i : L.path) {
			L2.path.add(i);
		}
		
		L2.unreachablePickupNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's unreachablePickupNodes to this label's unreachablePickupNodes
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new Vector<Integer>();
		// Adding all elements from the predecessor's openNodes to this label's openNodes
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		// Time in the label equals max of: 1) the predecessor's time plus travel- and service time to this node, 2) early time window in this node
		L2.time = Math.max(L.time+InstanceData.getTime(L.node, node, inputdata)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		
		// If the time is greater than the late time window of a node, return null
		if(L2.time > node.lateTimeWindow){
			return null;
		}
		
		// Deciding whether a pickup node is unreachable: if it is visited or if it is unreachable due to its time windows 	
		if(node.type == "PickupNode" && !L.unreachablePickupNodes.contains(node.number)){
			L2.unreachablePickupNodes.add(node.number); 
			L2.openNodes.add(node.number);
			for (int i=0 ; i < pickupNodes.size(); i++){
				if (!L2.path.contains(pickupNodes.get(i).number) && !L2.unreachablePickupNodes.contains(pickupNodes.get(i).number)){
					if (L2.time + InstanceData.getTime(node, pickupNodes.get(i), inputdata) + node.weight *inputdata.timeTonService > pickupNodes.get(i).lateTimeWindow){
						L2.unreachablePickupNodes.add(pickupNodes.get(i).number);
					}
				}
			}
		}
		
		// Removing the pickup node from the open nodes list if its corresponding delivery node is visited
		if (node.type == "DeliveryNode" && L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
			L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
		}

		// Adding the weight corresponding to a pickup node if the pickup node is visited and there is sufficient weight capacity on the vehicle 
		if(node.type=="PickupNode") {
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
		}
	
		// Removing the weight corresponding to a delivery node when the delivery node is visited
		if(node.type == "DeliveryNode") {
			L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
		}
		
		// Adding the volume corresponding to a pickup node if the pickup node is visited and there is sufficient volume capacity on the vehicle 
		if(node.type=="PickupNode") {
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
		}
		
		// Removing the volume corresponding to a delivery node when the delivery node is visited
		if(node.type == "DeliveryNode") {
			L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
		}
		
		// Calculating the profit (revenue - costs) when a pickup node is visited 
		if(node.type == "PickupNode") {
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes), inputdata))
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time);
			//System.out.println(L2.profit);
			//System.out.println(L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes), inputdata)));
			//System.out.println(inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata));
			//System.out.println(inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata));
			//System.out.println(inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata));
			//System.out.println((inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time));
			
		}
		
		// Calculating the profit (only costs) when a delivery node or the depots are visited
		if(node.type == "Depot" || node.type == "DeliveryNode") {
			L2.profit = L.profit - inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - L.time); 
		}
		L2.path.add(node.number);
		return L2;
	}
	
	
	
	public Vector<Label> BuildPaths() {
		Vector<Label> list = new Vector<Label>();  // List of non-dominated labels 
		Label L = new Label();
		// Initializing label
		L.labelNumber = 0;
		L.path = new Vector<Integer>();
		L.node = nodes.get(0);
		L.time = Float.parseFloat("20.4");
		L.profit = 0;
		L.weightCapacityUsed = 0;
		L.volumeCapacityUsed = 0;
		L.predesessor = null;
		L.unreachablePickupNodes = new Vector<Integer>();
		L.openNodes = new Vector<Integer>();		
		L.path.add(L.node.number);

		Vector<Label> processed = new Vector<Label>();
		PriorityQueue<Label> unprocessedQueue = new PriorityQueue<Label>(5, new UnprocessedComparator()); 
		unprocessedQueue.add(L);
		
		//Going through all unprocessed labels
		while(!unprocessedQueue.isEmpty()) { 
			Label label = unprocessedQueue.remove();
			for(int i = 2; i < nodes.size(); i++) { // Going through all nodes except node 0 and node 1 (the depot nodes)
				Label newLabel = LabelExtension(nodes.get(i), label);
				if(newLabel!=null) {
					if(checkdominance(newLabel, unprocessedQueue, processed)) {
						unprocessedQueue.add(newLabel); 
					}
				}
			}
			Label newLabel = LabelExtension(nodes.get(1), label); // Adding node 1 (the end depot node) to the end of the path 
			if(newLabel!=null) {
				if(checkdominance(newLabel, unprocessedQueue, processed)) {
					list.add(newLabel);
				}
			}
			processed.add(label); // The label removed from unprocessed is added to processed
		}
		
		System.out.println("Number of paths:" + processed.size());
		System.out.println("number of non-dominated paths: "+list.size());
		System.out.println("number of dominated labels: "+numberOfDominatedLabels);
		System.out.println("The best label is:");
		System.out.println(findBestLabel(list).toString());
		return list;
	}
	
	
	private boolean dominateLabel(Label L1, Label L2) {
		if(L1.time-zeroTol<=L2.time && L1.profit+zeroTol>=L2.profit && L1.node == L2.node) {
			for (int i : L1.openNodes ){
				if (!L2.openNodes.contains(i)){
					return false;
				}
			}
			for (int i : L1.unreachablePickupNodes ){
				if (!L2.unreachablePickupNodes.contains(i)){
					return false;
				}
			}
			//System.out.println("Label: ");
			//System.out.println(L1.toString());
			//System.out.println("dominates label: ");
			//System.out.println(L2.toString());
			return true;
		}
		else return false; 
	}
	
	
	
	//Updates the processed and unprocessed lists according to the dominated labels.
	private boolean checkdominance(Label newLabel, PriorityQueue<Label> unprocessedQueue, Vector<Label> processed) {
		Vector<Label> remove = new Vector<Label>();
		
		for(Label oldLabel : unprocessedQueue) {
			if(dominateLabel(oldLabel, newLabel)) {
				numberOfDominatedLabels++;
				unprocessedQueue.removeAll(remove);
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				remove.add(oldLabel);
				numberOfDominatedLabels++;
			}
		}
		unprocessedQueue.removeAll(remove);
		
		remove = new Vector<Label>();
		for(Label oldLabel : processed) {
			if(dominateLabel(oldLabel, newLabel)) {
				processed.removeAll(remove);
				numberOfDominatedLabels++;
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				numberOfDominatedLabels++;
				remove.add(oldLabel);
			}
		}
		processed.removeAll(remove);
		
		return true;
	}
	
	public Label findBestLabel(Vector<Label> list) {
		float currentBestProfit = 0;
		Label bestLabel = null;
		for(Label i : list) {
			if(i.profit > currentBestProfit) {
				currentBestProfit = i.profit;
				bestLabel = i;
			}
		}
		
		Label temp = bestLabel.predesessor;
		while(temp!=null) {
			System.out.println(temp.toString());
		temp=temp.predesessor;
		} 
		return bestLabel;
		
	}
	


}

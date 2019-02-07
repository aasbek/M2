import java.util.ArrayList;
//import java.util.Arrays;
import java.util.Hashtable;
//import java.util.Vector;
import java.util.Vector;

public class PathBuilder {
	public ArrayList<Node> nodes;
	public ArrayList<Node> pickupNodes;
	public ArrayList<Node> deliveryNodes;
	public ArrayList<Node> depot;
	public InstanceData inputdata;
	public ArrayList<Hashtable<Integer, Boolean>> feasibility;
	private double zeroTol = 0.001;
	private int numberOfDominatedLabels;
	
	public static ArrayList<Hashtable<Integer, Boolean>> feasibilityTest; 
	
	public PathBuilder(ArrayList<Node> pickupNodes, ArrayList<Node> deliveryNodes, ArrayList<Node> nodes, ArrayList<Node> depot, InstanceData inputdata) {
		this.pickupNodes = pickupNodes;
		this.nodes = nodes;
		this.deliveryNodes = deliveryNodes;
		this.depot = depot;
		this.inputdata = inputdata;
		feasibilityTest = new ArrayList<Hashtable<Integer, Boolean>>();
		numberOfDominatedLabels = 0;
		//System.out.println(nodes.get(2).locationName);
}
	
	
	
	
	public Label LabelExtension(Node node, Label L) {
		if(L.path.contains(node.number)) {
			return null;
		}
	
		Label L2 = new Label();
		L2.node = node;
		L2.predesessor = L;
		//L2.path = new ArrayList<Integer>();
		
		
	
	
		
		//L2.toString();

	
		
		//not return to start depot
		if(node.number == 0){
			return null;
		}
		
		//not leave end depot
		if (L.node.number == 1){
			return null;
		}
		
		L2.path = new ArrayList<Integer>();
		for(int i : L.path) {
			L2.path.add(i);
		}
		
		L2.unreachablePickupNodes = new ArrayList<Integer>();
		for(int i : L.unreachablePickupNodes) {
			L2.unreachablePickupNodes.add(i);
		}
		
		L2.openNodes = new ArrayList<Integer>();
		for(int i : L.openNodes) {
			L2.openNodes.add(i);
		}
		
		//not arrive at end depot without delivering every pickup
		if(node.number == 1 && !L.openNodes.isEmpty()){
			return null;	
		}
		//L2.path.add(node.number);
		
		//else {L2.path = L.path;
		//}
		//L.toString();
		
		L2.time = Math.max(L.time+InstanceData.getTime(L.node, node, inputdata)+L.node.weight*inputdata.timeTonService, node.earlyTimeWindow); 	
		
			
		if(node.type == "PickupNode" && !L.unreachablePickupNodes.contains(node.number)){
			L2.unreachablePickupNodes.add(node.number);
			L2.openNodes.add(node.number);
			for (int i=0 ; i < pickupNodes.size(); i++){
				
				if (!L2.path.contains(pickupNodes.get(i).number)){
					if (L2.time + InstanceData.getTime(node, pickupNodes.get(i), inputdata) + node.weight *inputdata.timeTonService > pickupNodes.get(i).lateTimeWindow){
						L2.unreachablePickupNodes.add(pickupNodes.get(i).number);
					}
				}
			}
		}
		//else return null;
		
		// only arrive at a delivery node if it has been picked up
		//if(node.type == "DeliveryNode"){
		//	System.out.println(node.number);
		//	System.out.println(node.getCorrespondingNode(node, nodes).number);
		//	if(!L.openNodes.contains(node.getCorrespondingNode(node,nodes))){
		//		System.out.println("2f");
		//		return null;//L2.openNodes.remove(node.getCorrespondingNode(node, nodes).number);
		//	}
			//else return null;
		//}
			
		
		
		if (node.type == "DeliveryNode" && ! L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){	
			return null;
		}
		
		
		if (node.type == "DeliveryNode" && L.openNodes.contains(node.getCorrespondingNode(node, nodes).number)){
			L2.openNodes.remove(L.openNodes.indexOf(node.getCorrespondingNode(node, nodes).number));
		}
		
		
		
	
		if(L2.time > node.lateTimeWindow){
			return null;
		}
//		if (L2.time < node.earlyTimeWindow){
//			L2.time = node.earlyTimeWindow;
//		}
		
		if(node.type=="PickupNode") {
			if(L.weightCapacityUsed + node.weight <= inputdata.weightCap){
				L2.weightCapacityUsed = L.weightCapacityUsed + node.weight;
			}
			else{
				return null;
			}
		}
		if(node.type == "DeliveryNode") {
			L2.weightCapacityUsed = L.weightCapacityUsed - node.weight;
		}
		if(node.type=="PickupNode") {
			if(L.volumeCapacityUsed + node.volume <= inputdata.volumeCap){
				L2.volumeCapacityUsed = L.volumeCapacityUsed + node.volume;
			}
			else{
				return null;
			}
		}
		if(node.type == "DeliveryNode") {
			L2.volumeCapacityUsed = L.volumeCapacityUsed - node.volume;
		}
		if(node.type == "PickupNode") {
			L2.profit = L.profit + (inputdata.revenue * node.weight * inputdata.getDistance(node, node.getCorrespondingNode(node, nodes), inputdata))
						- inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L2.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - 0);
			}
	
		
		if(node.type == "Depot" || node.type == "DeliveryNode") {
			L2.profit = L.profit - inputdata.fuelPrice*inputdata.fuelConsumptionEmptyTruckPerKm*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.fuelPrice*inputdata.fuelConsumptionPerTonKm*L2.weightCapacityUsed*inputdata.getDistance(L.node,node,inputdata)
						- inputdata.otherDistanceDependentCostsPerKm * inputdata.getDistance(L.node, node, inputdata)
						- (inputdata.laborCostperHour + inputdata.otherTimeDependentCostsPerKm)* (L2.time - 0); 
			}
		L2.path.add(node.number);
		//L2.toString();
		L2.labelNumber = L.labelNumber+1;
		return L2;
		
	}
	
	
	
	public ArrayList<Label> BuildPaths() {
		ArrayList<Label> list = new ArrayList<Label>();
		Label L = new Label();
		L.labelNumber = 0;
		L.path = new ArrayList<Integer>();
		L.node = nodes.get(0);
		L.time = 0;
		L.profit = 0;
		L.weightCapacityUsed = 0;
		L.volumeCapacityUsed = 0;
		L.predesessor = null;
		L.unreachablePickupNodes = new ArrayList<Integer>();
		L.openNodes = new ArrayList<Integer>();		
	
		L.path.add(L.node.number);
		//L.unreachablePickupNodes.add(L.node.number);
		//L.toString();
		
		ArrayList<Label> unprocessed = new ArrayList<Label>();
		ArrayList<Label> processed = new ArrayList<Label>();
		unprocessed.add(L);
		
		while(!unprocessed.isEmpty()) { //Going through all unprocessed labels
			//System.out.println("here");
			Label label = unprocessed.remove(0);
			for(int i = 2; i < nodes.size(); i++) {
			//System.out.println("here 2");
				Label newLabel = LabelExtension(nodes.get(i), label);
				//System.out.println(i);
				if(newLabel!=null) {
					if(checkdominance(newLabel, unprocessed, processed)) {
						unprocessed.add(newLabel); 
					}
				}
			}
			Label newLabel = LabelExtension(nodes.get(1), label);
			if(newLabel!=null) {
				if(checkdominance(newLabel, unprocessed, processed)) {
					list.add(newLabel);
				}
			}
			processed.add(label); //the label removed from unprosessed is added here
			
		
		}
		
		System.out.println("Number of paths:" + processed.size());
//		for(Label label : list) {
//			System.out.println(label.toString());
//		}
		System.out.println("number of non-dominated paths: "+list.size());
		System.out.println("number of dominated labels: "+numberOfDominatedLabels);
		return list;
	}
	
	
	private boolean dominateLabel(Label L1, Label L2) {
//		if(L1.openNodes.isEmpty() && L2.openNodes.isEmpty() && L1.path.contains(1)){
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
//			System.out.println("Label: ");
//			L1.toString();
//			System.out.println("dominates label: ");
//			L2.toString();
			return true;
		}
		else return false; 
//		}else return false;
	}
	
	
	
	//Updates the prosessed and unprossessed lists according to the dominated labels.
	private boolean checkdominance(Label newLabel, ArrayList<Label> unprocessed, ArrayList<Label> processed) {
		ArrayList<Label> remove = new ArrayList<Label>();
		
		for(Label oldLabel : unprocessed) {
			if(dominateLabel(oldLabel, newLabel)) {
				numberOfDominatedLabels++;
				unprocessed.removeAll(remove);
				return false;
			}
			else if(dominateLabel(newLabel,oldLabel)) {
				remove.add(oldLabel);
				numberOfDominatedLabels++;
			}
		}
		unprocessed.removeAll(remove);
		
		remove = new ArrayList<Label>();
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
	


}

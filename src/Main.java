import java.util.ArrayList;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String datafile = "1D10R1V.txt";
		//ArrayList<PickupNode> pickupnodes = new ArrayList<PickupNode>();
		//ArrayList<DeliveryNode> deliverynodes = new ArrayList<DeliveryNode>();
		ArrayList<Node> nodes = new ArrayList<Node>();
		ArrayList<Node> depot = new ArrayList<Node>();
		ArrayList<Node> pickupNodes = new ArrayList<Node>();
		ArrayList<Node> deliveryNodes = new ArrayList<Node>();
		
		InstanceData inputdata = new InstanceData(datafile);

		InputReader.inputReader(datafile, nodes, inputdata, depot, pickupNodes, deliveryNodes) ;

		//InstanceData.getDistance(nodes.get(1), nodes.get(5), inputdata);
		//InstanceData.getTime(nodes.get(4), nodes.get(5), inputdata);
		//System.out.println(nodes.get(0).location);
		//System.out.println(nodes.get(0).locationName);
		PathBuilder builder;
		builder = new PathBuilder(pickupNodes, deliveryNodes, nodes, depot,inputdata);
		builder.BuildPaths();
		//System.out.println(Node.getCorrespondingNode(nodes.get(2),nodes).number);
		
	}
}

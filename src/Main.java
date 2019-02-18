import java.util.ArrayList;
import java.util.Vector;

public class Main {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String datafile = "2D7R1V.txt";
		//ArrayList<PickupNode> pickupnodes = new ArrayList<PickupNode>();
		//ArrayList<DeliveryNode> deliverynodes = new ArrayList<DeliveryNode>();
		Vector<Node> nodes = new Vector<Node>();
		Vector<Node> depot = new Vector<Node>();
		Vector<Node> pickupNodes = new Vector<Node>();
		Vector<Node> deliveryNodes = new Vector<Node>();
		
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

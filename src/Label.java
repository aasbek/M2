
import java.util.ArrayList;


	public class Label {
		public int labelNumber;
		public float time; 
		public float profit;
		public float weightCapacityUsed;
		public float volumeCapacityUsed;
		public Label predesessor;
		public Node node;
		public ArrayList<Integer> path;
		public ArrayList<Integer> unreachablePickupNodes;
		public ArrayList<Integer> openNodes; //pickupnodes
		

	public String toString() {
		String string = "Node: " + node.number+ ", Time: " + time+", Profit: "+ profit + ", WeightCapacityUsed: " + weightCapacityUsed + ", VolumeCapacityUsed: " + volumeCapacityUsed+ 
				", Path: " +path+", unreachablePickupNodes: " + unreachablePickupNodes + ", openNodes: " + openNodes;
		//for (int  i : path) {
			//string += i;
		//}
//		Label temp = predesessor;
//		
//		while(temp!=null) {
//			string+=", Predessesor: "+temp.node.number;
//			temp=temp.predesessor;
//		}
//		
//		System.out.println(string);
//		System.out.println("");
//		System.out.println("");
		return string;
	}
}

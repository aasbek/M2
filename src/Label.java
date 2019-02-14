
import java.util.ArrayList;
import java.util.Vector;


	public class Label {
		public int labelNumber;
		public float time; 
		public float profit;
		public float weightCapacityUsed;
		public float volumeCapacityUsed;
		public Label predesessor;
		public Node node;
		public Vector<Integer> path;
		public Vector<Integer> unreachablePickupNodes;
		public Vector<Integer> openNodes; //pickupnodes
	//	public float waitingTime;
		

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

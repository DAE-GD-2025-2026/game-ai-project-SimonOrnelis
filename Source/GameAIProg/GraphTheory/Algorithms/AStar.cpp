#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};
	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};
	NodeRecord startNodeRecord{};
	NodeRecord currentNodeRecord{};
	
	// Start node
	startNodeRecord.pNode = pStartNode;
	startNodeRecord.pConnection = nullptr;
	// f(n) = g(n) + h(n)
	startNodeRecord.estimatedTotalCost = startNodeRecord.costSoFar + GetHeuristicCost(startNodeRecord.pNode, pGoalNode);
	
	// Add start node to open list
	openList.push_back(startNodeRecord);
	
	while (!openList.empty())
	{
		// Checks all nodes in openList and gets the one with the lowest cost
		currentNodeRecord = *std::min_element(openList.begin(), openList.end(), [](const NodeRecord& a, const NodeRecord& b){return a.estimatedTotalCost < b.estimatedTotalCost;});
		
		// Cheks if current node = goal node --> exit while loop
		if (currentNodeRecord.pNode == pGoalNode)
		{
			break;
		}
		else
		{
			// Goes over the connections of the current node 
			const auto& connections = pGraph->FindConnectionsFrom(currentNodeRecord.pNode->GetId());
			for (const auto node : connections)
			{
				Node* pNextNode = pGraph->GetNode(node->GetToId()).get();
				
				// Calc cost
				float newCostSoFar = currentNodeRecord.costSoFar + node->GetWeight(); // g
				float hCost = GetHeuristicCost(pNextNode, pGoalNode); // h
				float newEstimatedTotalCost = newCostSoFar + hCost; // f
				
				// Goes over closedList and checks if pNextNode is in it.
				auto closedIt = std::find_if(closedList.begin(), closedList.end(),[&](const NodeRecord& rec) { return rec.pNode == pNextNode; });
				
				// Checks if closedIt = end of closedList
				if (closedIt != closedList.end())
				{
					// Worse option → skip connection else remove
					if (newCostSoFar >= closedIt->costSoFar)
					{
						continue;  
					}
					else
					{
						closedList.erase(closedIt);
					}
				}
				
				auto openIt = std::find_if(openList.begin(), openList.end(),[&](const NodeRecord& rec) { return rec.pNode == pNextNode; });
				if (openIt != openList.end())
				{
					// Worse option → skip connection else remove
					if (newCostSoFar >= openIt->costSoFar)
					{
						continue;  
					}
					else
					{
						openList.erase(openIt);
					}
				}
				
				NodeRecord newNodeRecord{};
				newNodeRecord.pNode = pNextNode;
				newNodeRecord.pConnection = node;
				newNodeRecord.costSoFar = newCostSoFar;
				newNodeRecord.estimatedTotalCost = newEstimatedTotalCost;
				
				openList.push_back(newNodeRecord);
			}
			
			auto removeIt = std::find_if(
				openList.begin(), openList.end(),
				[&](const NodeRecord& rec) { return rec.pNode == currentNodeRecord.pNode; }
			);

			if (removeIt != openList.end())
				openList.erase(removeIt);

			closedList.push_back(currentNodeRecord);
		}
	}
	
	// safety
	path.clear();
	
	NodeRecord backtrackRecord = currentNodeRecord;
	
	// Add the goal node first
	path.push_back(backtrackRecord.pNode);
	
	
	while (backtrackRecord.pNode != pStartNode)
	{
		// Get the node ID of the previous node
		int previousNodeId = backtrackRecord.pConnection->GetFromId();
		Node* previousNode = pGraph->GetNode(previousNodeId).get();

		// Find the record of that node in CLOSED LIST
		auto prevIt = std::find_if(closedList.begin(), closedList.end(),[&](const NodeRecord& rec){return rec.pNode == previousNode;});

		if (prevIt == closedList.end())
		{
			// Should never happen if A* is correct safety
			break;
		}

		// Add the previous node to the path
		path.push_back(prevIt->pNode);

		// Move to previous record
		backtrackRecord = *prevIt;
	}
	
	std::reverse(path.begin(), path.end());
	
	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}
#include "BFS.h"

#include <map>
#include <queue>
#include <unordered_map>
#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;
	std::queue<Node*> openList{};
	std::vector<Node*> closedList{};
	std::unordered_map<Node*, Node*> previous;
	
	openList.push(pStartNode);
	
	bool destinationFound = false;
	
	while (!openList.empty())
	{
		auto currentNode = openList.front();
		openList.pop();
		
		// if end break
		if (currentNode == pDestinationNode)
		{
			destinationFound = true;
			break;
		}
		
		// Gets connections
		auto connections = pGraph->FindConnectionsFrom(currentNode->GetId());
		for (auto connection: connections)
		{
			Node* neighbor = pGraph->GetNode(connection->GetToId()).get();
			
			auto closedIt = std::find(closedList.begin(), closedList.end(),neighbor) != closedList.end();
			
			if (closedIt)
				continue;
			
			bool inOpenList = false;
			std::queue<Node*> temp = openList;
			while (!temp.empty())
			{
				if (temp.front() == neighbor)
				{
					inOpenList = true;
					break;
				}
				temp.pop();
			}
			
			if (inOpenList)
				continue;
			
			// Store parent (slide: "store previous node")
			previous[neighbor] = currentNode;

			// Add neighbor to openList (slide: "Add to Open List")
			openList.push(neighbor);

			// Mark as visited
			closedList.push_back(neighbor);
		}
	}
	
	// If no path found → return empty
	if (!destinationFound)
		return path;

	// --- Backtracking (slides page 12) ---
	Node* current = pDestinationNode;
	while (current != pStartNode)
	{
		path.push_back(current);
		current = previous[current]; // Go to parent
	}
	path.push_back(pStartNode);

	std::reverse(path.begin(), path.end());

	return path;
}

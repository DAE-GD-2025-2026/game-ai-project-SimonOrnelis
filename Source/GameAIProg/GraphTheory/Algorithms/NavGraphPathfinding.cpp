#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	//Create the path to return
	std::vector<FVector2D> finalPath{};

	//Get the start and endTriangle
	auto startTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(startPos, true);
	auto endTriangle = pNavGraph->GetNavPolygon()->GetTriangleAtPosition(endPos, true);
	
	if (startTriangle == nullptr && endTriangle == nullptr)
		return finalPath;
	
	if (startTriangle == endTriangle)
	{
		finalPath.reserve(2);
		finalPath.push_back(startPos);
		finalPath.push_back(endPos);
		return finalPath;
	}

	//We have valid start/end triangles, and they are not the same
	//=> Start looking for a path
	//Copy the graph
	auto graph = pNavGraph->Clone();

	//Create Extra node for the Start Node (Agent's position
	int startNodeId = graph->AddNode(std::make_unique<NavGraphNode>(startPos, -1));
	
	auto startEdges = startTriangle->GetEdges();
	for (const auto& edge : startEdges)
	{
		
		int edgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge).value_or(-1);
		if (edgeIdx < 0) 
			continue;
		
		int neighborNodeId = graph->GetNodeIdFromEdgeIndex(edgeIdx);
		if (neighborNodeId != Graphs::InvalidNodeId)
		{
			FVector2D posA = startPos;
			FVector2D posB = graph->GetNode(neighborNodeId)->GetPosition();
			//float cost = (posB - posA).Length();

			graph->AddConnection(startNodeId, neighborNodeId);
			graph->AddConnection(neighborNodeId, startNodeId);
		}
	}
	graph->SetConnectionCostsToDistances();
	
	//Create extra node for the endNode
	int endNodeId = graph->AddNode(std::make_unique<NavGraphNode>(endPos, -1));
	
	auto endEdges = endTriangle->GetEdges();
	for (const auto& edge : endEdges)
	{
		
		int edgeIdx = pNavGraph->GetNavPolygon()->FindEdgeIndex(edge).value_or(-1);
		if (edgeIdx < 0) 
			continue;
		
		int neighborNodeId = graph->GetNodeIdFromEdgeIndex(edgeIdx);
		if (neighborNodeId != Graphs::InvalidNodeId)
		{
			FVector2D posA = endPos;
			FVector2D posB = graph->GetNode(neighborNodeId)->GetPosition();
			//float cost = (posB - posA).Length();

			graph->AddConnection(endNodeId, neighborNodeId);
			graph->AddConnection(neighborNodeId, endNodeId);
		}
	}
	graph->SetConnectionCostsToDistances();
	
	//Run A star on new graph
	AStar astar(graph.get(), HeuristicFunctions::Manhattan);

	Node* startNode = graph->GetNode(startNodeId).get();
	Node* endNode   = graph->GetNode(endNodeId).get();

	std::vector<Node*> rawNodePath = astar.FindPath(startNode, endNode);
	if (rawNodePath.empty()) return finalPath;

	finalPath.reserve(rawNodePath.size());
	debugNodePositions.reserve(rawNodePath.size());


	// Extra: Run optimiser on new graph (First check if everything works without SSFA!)
	debugPortals = SSFA::FindPortals(rawNodePath, *pNavGraph->GetNavPolygon());
	finalPath = SSFA::OptimizePortals(debugPortals, *pNavGraph->GetNavPolygon());
	
	for (Node* n : rawNodePath)
	{
		FVector2D p = n->GetPosition();
		debugNodePositions.push_back(p);
		finalPath.push_back(p);
	}
	return finalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}
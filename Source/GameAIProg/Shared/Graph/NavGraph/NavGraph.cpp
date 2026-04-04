#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	auto const& edges = pNavPoly->GetEdges();
	auto const& triangles = pNavPoly->GetTriangles();
	
	//1. Go over all the edges of the navigation mesh and create nodes
	for (int edgeIdx = 0; edgeIdx < edges.size(); edgeIdx++)
	{
		const auto& edge = edges[edgeIdx];
		
		//check sharing triangle
		std::vector<int> sharingTriangles;
		sharingTriangles.reserve(2);
		for (int triIdx = 0; triIdx < triangles.size(); triIdx++)
		{
			if (triangles[triIdx].HasEdge(edge)) 
				sharingTriangles.push_back(triIdx);
		}
		
		//create graph node
		if (sharingTriangles.size() == 2)
		{
			FVector p1 = edge.GetP1(*pNavPoly);
			FVector p2 = edge.GetP2(*pNavPoly);
			FVector mid = (p1 + p2) * 0.5f;

			AddNode(std::make_unique<NavGraphNode>(FVector2D(mid.X, mid.Y), edgeIdx));
		}
	}
	
	//2. Create connections now that every node is created	
	for (int triIdx = 0; triIdx < triangles.size(); triIdx++)
	{
		auto edgesOfTri = triangles[triIdx].GetEdges();

		std::vector<int> nodeIds;

		// Find nodes
		for (auto const& edge : edgesOfTri)
		{
			int idx = pNavPoly->FindEdgeIndex(edge).value_or(-1);
			if (idx < 0) 
				continue;

			int foundNodeId = GetNodeIdFromEdgeIndex(idx);
			if (foundNodeId != Graphs::InvalidNodeId)
				nodeIds.push_back(foundNodeId);
		}

		// Connecting nodes
		if (nodeIds.size() == 2)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[0]);
		}
		else if (nodeIds.size() == 3)
		{
			AddConnection(nodeIds[0], nodeIds[1]);
			AddConnection(nodeIds[1], nodeIds[0]);

			AddConnection(nodeIds[1], nodeIds[2]);
			AddConnection(nodeIds[2], nodeIds[1]);

			AddConnection(nodeIds[0], nodeIds[2]);
			AddConnection(nodeIds[2], nodeIds[0]);
		}
	}
		
	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistances();
}

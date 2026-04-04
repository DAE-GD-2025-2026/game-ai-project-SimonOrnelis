#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "VectorTypes.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		//Container
		std::vector<NavLine> Portals = {};
		Portals.reserve(Path.size());
		
		//For each node received, get it's corresponding line
		for (int i{0}; i < Path.size() - 1; ++i)
		{
			auto* nodeA = dynamic_cast<NavGraphNode*>(Path[i]);	
			auto* nodeB = dynamic_cast<NavGraphNode*>(Path[i + 1]);
			
			if (!nodeA || !nodeB)
				continue;
			
			int edgeIdxA = nodeA->GetEdgeIdx();
			int edgeIdxB = nodeB->GetEdgeIdx();
			
			if (edgeIdxA >= 0 && edgeIdxA == edgeIdxB)
			{
				auto const& edge = NavPoly.GetEdges()[edgeIdxA];
				FVector p1 = edge.GetP1(NavPoly);
				FVector p2 = edge.GetP2(NavPoly);
				
				FVector2D v1{p1.X, p1.Y};
				FVector2D v2{p2.X, p2.Y};
				
				//Redetermine it's "orientation" based on the required path (left-right vs right-left) - p1 should be right point
				FVector2D pathDir = nodeB->GetPosition() - nodeA->GetPosition();
				FVector2D portalV = v1 - nodeA->GetPosition();
				
				float cross = Cross2D(pathDir, portalV);
				
				//Store portal
				NavLine portal{};
				if (cross < 0)
				{
					// v1 is right, v2 is left
					portal.P1 = v1;
					portal.P2 = v2;
				}
				else
				{
					// v2 is right, v1 is left
					portal.P1 = v2;
					portal.P2 = v1;
				}

				Portals.push_back(portal);
			}
		}
		//Add degenerate portal to force end evaluation
		if (!Portals.empty())
		{
			NavLine last = Portals.back();
			Portals.push_back(last);
		}

		return Portals;
	}

	static inline float Cross2D(const FVector2D& a, const FVector2D& b)
	{
		return a.X * b.Y - a.Y * b.X;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		std::vector<FVector2D> Path{};
		if (Portals.empty()) return Path;
		
		int portalCount = static_cast<int>(Portals.size());

		FVector2D apex = Path[0];
		FVector2D rightLeg = Portals[0].P1 - apex;
		FVector2D leftLeg  = Portals[0].P2 - apex;

		int apexIndex = 0;
		int rightIndex = 0;
		int leftIndex = 0;

		Path.push_back(apex);
		
		//P1 == right point of portal, P2 == left point of portal
		for (int i = 1; i < portalCount; i++)
		{
			const NavLine& portal = Portals[i];

			FVector2D newRight = portal.P1 - apex;
			FVector2D newLeft  = portal.P2 - apex;

			//--- RIGHT CHECK ---
			//1. See if moving funnel inwards - RIGHT
			if (Cross2D(rightLeg, newRight) <= 0)
			{
				//2. See if new line degenerates a line segment - RIGHT
				if (Cross2D(leftLeg, newRight) < 0)
				{
					//Leftleg becomes new apex point
					apex += leftLeg;
					Path.push_back(apex);

					//Calculate new legs (if not the end)
					apexIndex = leftIndex;
					i = apexIndex + 1;

					rightIndex = i;
					leftIndex  = i;

					if (i < portalCount)
					{
						rightLeg = Portals[i].P1 - apex;
						leftLeg  = Portals[i].P2 - apex;
					}
					continue;
				}
				else
				{
					rightLeg = newRight;
					rightIndex = i;
				}
			}

			//--- LEFT CHECK ---
			//1. See if moving funnel inwards - LEFT
			if (Cross2D(leftLeg,newLeft) <= 0)
			{
				//2. See if new line degenerates a line segment - LEFT
				if (Cross2D(newLeft,rightLeg) < 0)
				{
					//Rightleg becomes new apex point
					apex += rightLeg;
					Path.push_back(apex);

					apexIndex = rightIndex;
					i = apexIndex + 1;

					rightIndex = i;
					leftIndex  = i;

					if (i < portalCount)
					{
						rightLeg = Portals[i].P1 - apex;
						leftLeg  = Portals[i].P2 - apex;
					}
					continue;
				}
					//Calculate new legs (if not the end)
				else
				{
					// Tighten left
					leftLeg = newLeft;
					leftIndex = i;
				}
			}
		}
		// Add last path point
		Path.push_back(Portals.back().P2);
		return Path;
	}
private:
	SSFA() {};
	~SSFA() {};
};
}

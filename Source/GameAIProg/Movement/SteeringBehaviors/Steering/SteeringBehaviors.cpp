#include "SteeringBehaviors.h"

#include "DynamicMesh/DynamicMesh3.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed());
	
	FVector TargetPos{Target.Position.X,Target.Position.Y,0.f};

	// Goes to the mouse position
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(),TargetPos,5,8,FColor::Red,false,0.f,0,5.f,FVector::RightVector,FVector::ForwardVector,false);
	}
	
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed());

	// Goes away from the mouse position
	Steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	
	return Steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	FVector Center{Agent.GetPosition().X,Agent.GetPosition().Y,0.f};
	FVector TargetPos{Target.Position.X,Target.Position.Y,0.f};
	float SlowRadius{500.f}; 
	//float TargetRadius{150.f};
	FVector2D ToTarget = Target.Position - Agent.GetPosition();
	float Distance = ToTarget.Length();
	float TargetSpeed = Agent.GetOriginalMaxLinearSpeed();
	
	if (Distance < m_TargetRadius)
	{
		return Steering;
	}
	
	if (Distance < SlowRadius)
	{
		const float t = (Distance - m_TargetRadius) / FMath::Max(SlowRadius - m_TargetRadius, KINDA_SMALL_NUMBER);
		TargetSpeed *= FMath::Clamp(t, 0.f, 1.f);
	}
	Agent.SetMaxLinearSpeed(TargetSpeed);
	
	// Goes to the mouse position
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	if (Agent.GetDebugRenderingEnabled())
	{
		// End point
		DrawDebugCircle(Agent.GetWorld(),TargetPos,5,8,FColor::Red,false,0.f,0,5.f,FVector::RightVector,FVector::ForwardVector,false);
		
		// Radii
		DrawDebugCircle(Agent.GetWorld(),Center,SlowRadius,16,FColor::Blue,false,0.f,0,3.f,FVector::RightVector,FVector::ForwardVector,false);
		DrawDebugCircle(Agent.GetWorld(),Center,m_TargetRadius,16,FColor::Red,false,0.f,0,3.f,FVector::RightVector,FVector::ForwardVector,false);
	}
	
	return Steering;
}

void Arrive::SetTargetRadius(float radius)
{
	m_TargetRadius = radius;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	//Agent.SetMaxLinearSpeed(0.f);
	
	return Seek::CalculateSteering(DeltaT, Agent);
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	Agent.SetMaxLinearSpeed(Agent.GetOriginalMaxLinearSpeed());
	
	// Generate random angle
	float Angle = FMath::RandRange(0.0f, 2.0f * PI);
	float Radius = 100.f;
	FVector Center{
		Agent.GetPosition().X + Agent.GetLinearVelocity().X * 0.250f,
		Agent.GetPosition().Y + Agent.GetLinearVelocity().Y * 0.250f,
		0.f
	};
	
	FVector2D RandomPointOnEdge;
	RandomPointOnEdge.X = FMath::Cos(Angle) * Radius + Center.X;
	RandomPointOnEdge.Y = FMath::Sin(Angle) * Radius + Center.Y;
	
	// Set Target pos to the random point on the edge;
	Target.Position = RandomPointOnEdge;
	
	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(Agent.GetWorld(),Center,Radius,16,FColor::Blue,false,0.f,0,3.f,FVector::RightVector,FVector::ForwardVector,false);
	}
	 
	return Seek::CalculateSteering(DeltaT, Agent);
}

#include "GAPathComponent.h"
#include "GameFramework/NavMovementComponent.h"
#include "Kismet/GameplayStatics.h"

UGAPathComponent::UGAPathComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	State = GAPS_None;
	bDestinationValid = false;
	ArrivalDistance = 100.0f;

	// A bit of Unreal magic to make TickComponent below get called
	PrimaryComponentTick.bCanEverTick = true;
}


const AGAGridActor* UGAPathComponent::GetGridActor() const
{
	if (GridActor.Get())
	{
		return GridActor.Get();
	}
	else
	{
		AGAGridActor* Result = NULL;
		AActor *GenericResult = UGameplayStatics::GetActorOfClass(this, AGAGridActor::StaticClass());
		if (GenericResult)
		{
			Result = Cast<AGAGridActor>(GenericResult);
			if (Result)
			{
				// Cache the result
				// Note, GridActor is marked as mutable in the header, which is why this is allowed in a const method
				GridActor = Result;
			}
		}

		return Result;
	}
}

APawn* UGAPathComponent::GetOwnerPawn()
{
	AActor* Owner = GetOwner();
	if (Owner)
	{
		APawn* Pawn = Cast<APawn>(Owner);
		if (Pawn)
		{
			return Pawn;
		}
		else
		{
			AController* Controller = Cast<AController>(Owner);
			if (Controller)
			{
				return Controller->GetPawn();
			}
		}
	}

	return NULL;
}


void UGAPathComponent::TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (bDestinationValid)
	{
		RefreshPath();

		if (State == GAPS_Active)
		{
			FollowPath();
		}
	}

	// Super important! Otherwise, unbelievably, the Tick event in Blueprint won't get called

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

EGAPathState UGAPathComponent::RefreshPath()
{
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();
	check(bDestinationValid);

	float DistanceToDestination = FVector::Dist(StartPoint, Destination);

	if (DistanceToDestination <= ArrivalDistance)
	{
		// Yay! We got there!
		State = GAPS_Finished;
	}
	else
	{
		TArray<FPathStep> UnsmoothedSteps;

		Steps.Empty();

		// Replan the path!
		State = AStar(StartPoint, UnsmoothedSteps);

		// To debug A* without smoothing:
		//Steps = UnsmoothedSteps;

		if (State == EGAPathState::GAPS_Active)
		{
			// Smooth the path!
			State = SmoothPath(StartPoint, UnsmoothedSteps, Steps);
		}
	}

	return State;
}
float Heuristic(const FCellRef& A, const FCellRef& B) 
{
	return FMath::Abs(A.X - B.X) + FMath::Abs(A.Y - B.Y);
}

TArray<FCellRef> GetNeighbors(const FCellRef& CurrentCell) 
{
	TArray<FCellRef> Neighbors;
	Neighbors. Add(FCellRef(CurrentCell.X + 1, CurrentCell.Y));
	Neighbors. Add(FCellRef(CurrentCell.X - 1, CurrentCell.Y));
	Neighbors. Add(FCellRef(CurrentCell.X, CurrentCell.Y + 1));
	Neighbors. Add (FCellRef(CurrentCell.X, CurrentCell.Y - 1));
	return Neighbors;
}

//Function to create Array of Fpathsteps from startcell to destination cell
void ReconstructPath(const TMap<FCellRef, FCellRef>& CameFrom, FCellRef Current, const AGAGridActor* Grid, TArray<FPathStep>& StepsOut) 
{
	StepsOut.Empty(); 

	TArray<FCellRef> Path;
	Path.Add(Current);

	while (CameFrom.Contains(Current))
	{
		Current = CameFrom[Current]; 
		Path.Insert(Current, 0); 
	}
Path.Remove(Current);
	
	for (const FCellRef& Cell : Path)
	{
		FVector WorldLocation = Grid->GetCellPosition(Cell); 

		FPathStep Step;
		Step.Set(WorldLocation, Cell); 
		StepsOut.Add(Step);
	}
}

EGAPathState UGAPathComponent::AStar(const FVector& StartPoint, TArray<FPathStep>& StepsOut) const
{
	const AGAGridActor* Grid = GetGridActor();
    
    // Ensure grid exists
	if (!Grid)
	{
		return GAPS_Invalid; 
	}

	FCellRef StartCell = Grid->GetCellRef(StartPoint);
	TArray<FCellRef> OpenSet;
	TMap<FCellRef, FCellRef> CameFrom; 
	TMap<FCellRef, float> GScore;     
	TMap<FCellRef, float> FScore;   

	// GScore of the Start Cell is 0.
	GScore.Add(StartCell, 0);

	// FScore = GScore + H. Since GScore=0, FScore[start_cell] = Heuristic
	FScore.Add(StartCell, Heuristic(StartCell, DestinationCell));

	// Override the comparator so that it adds Cells to heap ans sorts based on lower FScore
	OpenSet.HeapPush(StartCell, [&FScore](const FCellRef& A, const FCellRef& B) {
		return FScore[A] < FScore[B]; 
	});

	//Loop until we finally reach the target i.e. the bot catches the main character
	while (OpenSet.Num() > 0)
	{
		FCellRef Current;
		OpenSet.HeapPop(Current, [&FScore](const FCellRef& A, const FCellRef& B) {
			return FScore[A] < FScore[B]; 
		});

		//If we find the destination, we need to end our A* algorithm and reconstruct this path from the start cell to the destination cell.
		if (Current == DestinationCell)
		{
			
			ReconstructPath(CameFrom, Current,Grid, StepsOut);
			return GAPS_Active;
		}

		//Until we find destination, we need to explore the neighbors of the current cell. We explore cell as long as it isnt out of bounds or a non traversable cell (wall/box)
		TArray<FCellRef> Neighbors = GetNeighbors(Current);
		for (const FCellRef& Neighbor : Neighbors)
		{
			ECellData Flags = Grid->GetCellData(Neighbor);

			if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable) || !Grid->IsCellRefInBounds(Neighbor))
			{
				continue; 
			}
			float TentativeGScore = GScore[Current] + 1; 

			if (!GScore.Contains(Neighbor) || TentativeGScore < GScore[Neighbor])
			{
				CameFrom.Add(Neighbor, Current);
				GScore.Add(Neighbor, TentativeGScore);
				//FScore = GScore + H
				FScore.Add(Neighbor, TentativeGScore + Heuristic(Neighbor, DestinationCell));

				if (!OpenSet.Contains(Neighbor))
				{
					OpenSet.HeapPush(Neighbor, [&FScore](const FCellRef& A, const FCellRef& B) {
						return FScore[A] < FScore[B];
					});
				}
			}
		}

	}

	

	// Assignment 2 Part3: replace this with an A* search!
	// HINT 1: you made need a heap structure. A TArray can be accessed as a heap -- just add/remove elements using
	// the TArray::HeapPush() and TArray::HeapPop() methods.
	// Note that whatever you push or pop needs to implement the 'less than' operator (operator<)
	// HINT 2: UE has some useful flag testing function. For example you can test for traversability by doing this:
	// ECellData Flags = Grid->GetCellData(CellRef);
	// bool bIsCellTraversable = EnumHasAllFlags(Flags, ECellData::CellDataTraversable)

	StepsOut.SetNum(1);
	StepsOut[0].Set(Destination, DestinationCell);
	
	
	return GAPS_Active;
}


bool LineTrace(const FVector& Start, const FVector& End, const AGAGridActor* Grid) 
{
	if (!Grid)
	{
		return false; 
	}

	const FCellRef StartCell = Grid->GetCellRef(Start);
	const FCellRef EndCell = Grid->GetCellRef(End);

	if (!Grid->IsCellRefInBounds(StartCell) || !Grid->IsCellRefInBounds(EndCell))
	{
		return true; 
	}

	
	FVector Direction = End - Start;
	const float Distance = Direction.Size();
	Direction.Normalize();  

	// Traverse from start cell in ditection of end cell with a step size of 10 units. 
	for (float i = 0; i < Distance; i += 10)  
	{
		FVector CurrentPosition = Start + Direction * i;

		FCellRef CurrentCell = Grid->GetCellRef(CurrentPosition);

		ECellData Flags = Grid->GetCellData(CurrentCell);

		if (!EnumHasAllFlags(Flags, ECellData::CellDataTraversable))
		{
			return true; 
		}
	}

	return false;
}

EGAPathState UGAPathComponent::SmoothPath(const FVector& StartPoint, const TArray<FPathStep>& UnsmoothedSteps, TArray<FPathStep>& SmoothedStepsOut) const
{
	SmoothedStepsOut.Empty();

	if (UnsmoothedSteps.Num() == 0)
	{
		return GAPS_Invalid; 
	}
	// grid invald
	const AGAGridActor* Grid = GetGridActor();
	if (!Grid)
	{
		return GAPS_Invalid; 
	}

	FPathStep CurrentStep =  UnsmoothedSteps[0];
	if (!LineTrace(StartPoint, Grid->GetCellPosition(UnsmoothedSteps.Last().CellRef), Grid))
	{
		SmoothedStepsOut.Add(UnsmoothedSteps.Last());
		return GAPS_Active;

	}

	while (true)
	{
		SmoothedStepsOut.Add(CurrentStep);

		// Destination reached
		if (CurrentStep.CellRef == UnsmoothedSteps.Last().CellRef)
		{
			break;
		}

		int32 NextIndex = INDEX_NONE;
		
		for (int32 i = SmoothedStepsOut.Num(); i < UnsmoothedSteps.Num(); i++)
		{
			FVector CurrentPosition = Grid->GetCellPosition(CurrentStep.CellRef);

			if (LineTrace(CurrentPosition, Grid->GetCellPosition(UnsmoothedSteps[i].CellRef), Grid))
			{
				NextIndex = i - 1; 
				break;
			}
		}

		if (NextIndex == INDEX_NONE)
		{
			NextIndex = UnsmoothedSteps.Num() - 1;
		}

		CurrentStep.Set(Grid->GetCellPosition(UnsmoothedSteps[NextIndex].CellRef), UnsmoothedSteps[NextIndex].CellRef);
	}

	return GAPS_Active;
}


void UGAPathComponent::FollowPath()
{
	AActor* Owner = GetOwnerPawn();
	FVector StartPoint = Owner->GetActorLocation();

	check(State == GAPS_Active);
	check(Steps.Num() > 0);

	// Always follow the first step, assuming that we are refreshing the whole path every tick
	FVector V = Steps[0].Point - StartPoint;
	V.Normalize();

	UNavMovementComponent* MovementComponent = Owner->FindComponentByClass<UNavMovementComponent>();
	if (MovementComponent)
	{
		MovementComponent->RequestPathMove(V);
	}
}


EGAPathState UGAPathComponent::SetDestination(const FVector &DestinationPoint)
{
	Destination = DestinationPoint;

	State = GAPS_Invalid;
	bDestinationValid = true;

	const AGAGridActor* Grid = GetGridActor();
	if (Grid)
	{
		FCellRef CellRef = Grid->GetCellRef(Destination);
		if (CellRef.IsValid())
		{
			DestinationCell = CellRef;
			bDestinationValid = true;

			RefreshPath();
		}
	}

	return State;
}
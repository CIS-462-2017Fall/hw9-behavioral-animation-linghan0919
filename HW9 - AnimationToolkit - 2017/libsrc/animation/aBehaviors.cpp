#include "aBehaviors.h"

#include <math.h>
#include "GL/glew.h"
#include "GL/glut.h"

#ifndef RAD
#define PI 3.14159265358979f
#define RAD (PI / 180.0f)
#endif

// Base Behavior
///////////////////////////////////////////////////////////////////////////////
Behavior::Behavior()
{
}

Behavior::Behavior( char* name) 
{
	m_name = name;
	m_pTarget = NULL;
}

Behavior::Behavior( Behavior& orig) 
{
	m_name = orig.m_name;
	m_pTarget = NULL;
}

string& Behavior::GetName() 
{
    return m_name;
}

// Behaviors derived from Behavior
//----------------------------------------------------------------------------//
// Seek behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Seek returns a maximum velocity towards the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position


Seek::Seek( AJoint* target) 
{
	m_name = "seek";
	m_pTarget = target;

}

Seek::Seek( Seek& orig) 
{
	m_name = "seek";
	m_pTarget = orig.m_pTarget;
}


Seek::~Seek()
{
}

vec3 Seek::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	Vdesired = targetPos - actorPos;
	Vdesired.Normalize();
	Vdesired = Vdesired * actor->gMaxSpeed;
	// Linghan 2017-12-04

	return Vdesired;
}


// Flee behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Flee calculates a a maximum velocity away from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position

Flee::Flee( AJoint* target) 
{
	m_name = "flee";
	m_pTarget = target;
}

Flee::Flee( Flee& orig) 
{
	m_name = "flee";
	m_pTarget = orig.m_pTarget;
}

Flee::~Flee()
{
}

vec3 Flee::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	Vdesired = actorPos - targetPos;
	Vdesired.Normalize();
	Vdesired = Vdesired * actor->gMaxSpeed;
	// Linghan 2017-12-04

	return Vdesired;

}

// Arrival behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// the actors distance from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Arrival strength is in BehavioralController::KArrival


Arrival::Arrival( AJoint* target) 
{
	m_name = "arrival";
	m_pTarget = target;
}

Arrival::Arrival( Arrival& orig) 
{
	m_name = "arrival";
	m_pTarget = orig.m_pTarget;
}

Arrival::~Arrival()
{
}

vec3 Arrival::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 e = targetPos - actorPos;
	Vdesired = actor->KArrival * e;
	// Linghan 2017-12-04

	return Vdesired;
}


// Departure behavior
///////////////////////////////////////////////////////////////////////////////
// Given the actor, return a desired velocity in world coordinates
// Arrival returns a desired velocity vector whose speed is proportional to
// 1/(actor distance) from the target
// m_pTarget contains target world position
// actor.getPosition() returns Agent's world position
//  Departure strength is in BehavioralController::KDeparture

Departure::Departure(AJoint* target) 
{
	m_name = "departure";
	m_pTarget = target;
}

Departure::Departure( Departure& orig) 
{
	m_name = "departure";
	m_pTarget = orig.m_pTarget;
}

Departure::~Departure()
{
}

vec3 Departure::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();

	// TODO: add your code here to compute Vdesired
	vec3 e = targetPos - actorPos;
	Vdesired = - (actor->KDeparture * e) / (e.Length() * e.Length());
	// Linghan 2017-12-04

	return Vdesired;
}


// Avoid behavior
///////////////////////////////////////////////////////////////////////////////
//  For the given the actor, return a desired velocity in world coordinates
//  If an actor is near an obstacle, avoid adds a normal response velocity to the 
//  the desired velocity vector computed using arrival
//  Agent bounding sphere radius is in BehavioralController::radius
//  Avoidance parameters are  BehavioralController::TAvoid and BehavioralController::KAvoid

Avoid::Avoid(AJoint* target, vector<Obstacle>* obstacles) 
{
	m_name = "avoid";
	m_pTarget = target;
	mObstacles = obstacles;
}

Avoid::Avoid( Avoid& orig) 
{
	m_name = "avoid";
	m_pTarget = orig.m_pTarget;
	mObstacles = orig.mObstacles;
}

Avoid::~Avoid()
{
}

vec3 Avoid::calcDesiredVel( BehaviorController* actor)
{

	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	m_actorPos = actor->getPosition();
	m_actorVel = actor->getVelocity();

	//TODO: add your code here
	vec3 Varrival(0, 0, 0);
	// Step 1. compute initial value for Vdesired = Varrival so agent moves toward target
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vec3 e = targetPos - actorPos;
	Vdesired = actor->KArrival * e;

	vec3 Vavoid(0, 0, 0);
	//TODO: add your code here to compute Vavoid 
	// Step 2. compute Lb
	//TODO: add your code here
	double LB = actor->TAvoid * m_actorVel.Length();

	// Step 3. find closest obstacle 
	//TODO: add your code here
	double min_dist = 10000.0;
	int min_obstacle = -1;
	for (int i = 0; i < mObstacles->size(); i++) {
		Obstacle p_obstacle = mObstacles->at(i);
		vec3 obsPos = p_obstacle.m_Center.getLocalTranslation();
		vec3 d = obsPos - m_actorPos;
		// Dot(m_actorVel, d) >= 0 is used to make sure the obstacle is in front of agent
		if (Dot(m_actorVel, d) >= 0 && d.Length() < min_dist) { 
			min_dist = d.Length();
			min_obstacle = i;
		}
	}
	if (min_obstacle == -1) return Vdesired; // no obstacle need to worry about

	// Step 4. determine whether agent will collide with closest obstacle (only consider obstacles in front of agent)
	//TODO: add your code here
	Obstacle p_obstacle = mObstacles->at(min_obstacle);
	m_obstaclePos = p_obstacle.m_Center.getLocalTranslation();
	vec3 d_world = m_obstaclePos - m_actorPos;
	vec3 body_z = m_actorVel; body_z.Normalize();
	vec3 d_z = Dot(d_world, body_z);
	vec3 d_x = d_world - d_z;
	if (d_z.Length() > LB + actor->gAgentRadius + p_obstacle.m_Radius) return Vdesired; // no collision
	if (d_x.Length() > actor->gAgentRadius + p_obstacle.m_Radius) return Vdesired; // no collision

	// Step 5.  if potential collision detected, compute Vavoid and set Vdesired = Varrival + Vavoid
	//TODO: add your code here
	vec3 n_avoid = -d_x / d_x.Length();
	double vAvoid_len = (actor->KAvoid * (actor->gAgentRadius + p_obstacle.m_Radius - d_x.Length())) / 
						(actor->gAgentRadius + p_obstacle.m_Radius);
	vec3 vAvoid = n_avoid * vAvoid_len;
	Vdesired = Vdesired + vAvoid;
	//std::cout << "try to avoid" << std::endl;
	// Linghan 2017-12-04
	return Vdesired;
	
}

void Avoid::display( BehaviorController* actor)
{
	//  Draw Debug info
	vec3 angle = actor->getOrientation();
	vec3 vel = actor->getVelocity();
	vec3 dir = vec3(cos(angle[1]), 0, sin(angle[1]));
	vec3 probe = dir * (vel.Length()/BehaviorController::gMaxSpeed)*BehaviorController::TAvoid;
	
	glBegin(GL_LINES);
	glColor3f(0, 0, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_obstaclePos[0], m_obstaclePos[1], m_obstaclePos[2]);
	glColor3f(0, 1, 1);
	glVertex3f(m_actorPos[0], m_actorPos[1], m_actorPos[2]);
	glVertex3f(m_actorPos[0] + probe[0], m_actorPos[1] + probe[1], m_actorPos[2] + probe[2]);
	glEnd();
}


// Wander Behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Wander returns a desired velocity vector whose direction changes at randomly from frame to frame
// Wander strength is in BehavioralController::KWander

Wander::Wander() 
{
	m_name = "wander";
	m_Wander = vec3(1.0, 0.0, 0.0);
}

Wander::Wander( Wander& orig) 
{
	m_name = "wander";
	m_Wander = orig.m_Wander;
}

Wander::~Wander()
{
}

vec3 Wander::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();

	// compute Vdesired = Vwander

	// Step. 1 find a random direction
	//TODO: add your code here
	double theta = -180.0 + rand() % 360;
	vec3 noise = vec3(cos(theta * RAD), 0.0, sin(theta * RAD));
	noise.Normalize();

	// Step2. scale it with a noise factor
	//TODO: add your code here
	vec3 r_noise = actor->KNoise * noise;

	// Step3. change the current Vwander to point to a random direction
	//TODO: add your code here
	m_Wander = actor->KWander * (m_Wander + r_noise).Normalize();
	
	// Step4. scale the new wander velocity vector and add it to the nominal velocity
	//TODO: add your code here
	Vdesired = actor->getVelocity() + m_Wander;
	// Linghan 2017-12-04

	return Vdesired;
}


// Alignment behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity vector in world coordinates
// Alignment returns the average velocity of all active agents in the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Alignment parameters are in BehavioralController::RNeighborhood and BehavioralController::KAlign


Alignment::Alignment(AJoint* target, vector<AActor>* agents) 
{
	m_name = "alignment";
	m_pAgentList = agents;
	m_pTarget = target;
}



Alignment::Alignment( Alignment& orig) 
{
	m_name = orig.m_name;
	m_pAgentList = orig.m_pAgentList;
	m_pTarget = orig.m_pTarget;

}

Alignment::~Alignment()
{
}

vec3 Alignment::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_pAgentList;
	

	// compute Vdesired 
	
	// Step 1. compute value of Vdesired for fist agent (i.e. m_AgentList[0]) using an arrival behavior so it moves towards the target
	 
	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader
	//TODO: add your code here
	if (actor == leader) {
		vec3 e = targetPos - actorPos;
		Vdesired = actor->KArrival * e;
		return Vdesired;
	}
	// Linghan 2017-12-05

	// Step 2. if not first agent compute Valign as usual
	//TODO: add your code here
	vec3 gain = vec3(0.0, 0.0, 0.0);
	double omega = 0.0;
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* neigh_agent = agentList[i].getBehaviorController();
		vec3 dist = neigh_agent->getPosition() - actorPos;
		if (dist.Length() <= actor->gKNeighborhood) {
			gain = gain + 5.0 * neigh_agent->getVelocity(); 
			omega += 5.0; // assume all w_i are the same here -> 5
		}

		// assume all w_i are the same here
	}
	if (gain == vec3Zero)
		Vdesired = actor->getVelocity() / 10; // slow down
	else
		Vdesired = actor->KAlignment * (gain / omega);
	// Linghan 2017-12-05
	
	return Vdesired;
}

// Separation behavior
///////////////////////////////////////////////////////////////////////////////
// For the given te actor, return a desired velocity vector in world coordinates
// Separation tries to maintain a constant distance between all agents
// within the neighborhood
// agents[i] gives the pointer to the ith agent in the environment
// Separation settings are in BehavioralController::gKNeighborhood and BehavioralController::KSeperate

 

Separation::Separation( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "separation";
	m_AgentList = agents;
	m_pTarget = target;
}

Separation::~Separation()
{
}

Separation::Separation( Separation& orig) 
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

vec3 Separation::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vseparate
	// TODO: add your code here to compute Vdesired 
	vec3 gain = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* neigh_agent = agentList[i].getBehaviorController();
		if (neigh_agent == actor) continue;
		vec3 dist = actorPos - neigh_agent->getPosition();
		if (dist.Length() <= actor->gKNeighborhood) 
			 gain = gain +  5 * dist / (dist.Length() * dist.Length()); 
			// assume all w_i are the same here -> 5
	}
	if (gain == vec3Zero)
		Vdesired = actor->getVelocity() / 10; // slow down
	else
		Vdesired = actor->KSeparation * gain;
	// Linghan 2017-12-05

	//if (Vdesired.Length() < 5.0)
	//	Vdesired = 0.0;
	return Vdesired;
}


// Cohesion behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// Cohesion moves actors towards the center of the group of agents in the neighborhood
//  agents[i] gives the pointer to the ith agent in the environment
//  Cohesion parameters are in BehavioralController::RNeighborhood and BehavioralController::KCohesion


Cohesion::Cohesion( vector<AActor>* agents) 
{
	m_name = "cohesion";
	m_AgentList = agents;
}

Cohesion::Cohesion( Cohesion& orig) 
{
	m_name = "cohesion";
	m_AgentList = orig.m_AgentList;
}

Cohesion::~Cohesion()
{
}

vec3 Cohesion::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;
	
	// compute Vdesired = Vcohesion
	// TODO: add your code here to compute Vdesired 
	vec3 posSum = vec3(0.0, 0.0, 0.0);
	double omega = 0.0;
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* neigh_agent = agentList[i].getBehaviorController();
		vec3 neighPos = neigh_agent->getPosition();
		vec3 dist = neighPos - actorPos;
		if (dist.Length() <= actor->gKNeighborhood) {
			posSum = posSum + 5.0 * neighPos;
			omega += 5.0; // assume all w_i are the same
		}
	}
	if (posSum == vec3Zero)
		Vdesired = actor->getVelocity() / 10; // slow down
	else
		Vdesired = actor->KCohesion * (posSum / omega - actorPos) ;
	// Linghan 2017-12-05 
	

	return Vdesired;
}

// Flocking behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector  in world coordinates
// Flocking combines separation, cohesion, and alignment behaviors
//  Utilize the Separation, Cohesion and Alignment behaviors to determine the desired velocity vector


Flocking::Flocking( AJoint* target,  vector<AActor>* agents) 
{
	m_name = "flocking";
	m_AgentList = agents;
	m_pTarget = target;
}

Flocking::Flocking( Flocking& orig) 
{
	m_name = "flocking";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Flocking::~Flocking()
{
}

vec3 Flocking::calcDesiredVel( BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// compute Vdesired = Vflocking
	// TODO: add your code here 
	Behavior* sep = new Separation(actor->getTarget(), m_AgentList);
	vec3 vSeparate = sep->calcDesiredVel(actor);
	Behavior* align = new Alignment(actor->getTarget(), m_AgentList);
	vec3 vAlign = align->calcDesiredVel(actor);
	Behavior* coh = new Cohesion(m_AgentList);
	vec3 vCohesion = coh->calcDesiredVel(actor);

	double CSep = 0.2;
	double CAlign = 0.6;
	double CCoh = 0.2;

	Vdesired = CSep * vSeparate + CAlign * vAlign + CCoh * vCohesion;
	// Linghan 2017-12-05

	return Vdesired;
}

//	Leader behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// If the agent is the leader, move towards the target; otherwise, 
// follow the leader at a set distance behind the leader without getting to close together
//  Utilize Separation and Arrival behaviors to determine the desired velocity vector
//  You need to find the leader, who is always agents[0]

Leader::Leader( AJoint* target, vector<AActor>* agents) 
{
	m_name = "leader";
	m_AgentList = agents;
	m_pTarget = target;
}

Leader::Leader( Leader& orig) 
{
	m_name = "leader";
	m_AgentList = orig.m_AgentList;
	m_pTarget = orig.m_pTarget;
}

Leader::~Leader()
{
}

vec3 Leader::calcDesiredVel( BehaviorController* actor)
{
	
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vec3 targetPos = m_pTarget->getLocalTranslation();
	vector<AActor>& agentList = *m_AgentList;

	// TODO: compute Vdesired  = Vleader
	// followers should stay directly behind leader at a distance of -200 along the local z-axis

	float CSeparation = 0.5;  float CArrival = 1.0;

	BehaviorController* leader = agentList[0].getBehaviorController(); // first agent is the leader

	if (actor == leader) { // leader follows target
		vec3 e = targetPos - actorPos;
		Vdesired = actor->KArrival * e;
	}
	else { // others follow leader
		Behavior* sep = new Separation(actor->getTarget(), m_AgentList);
		vec3 vSeparate = sep->calcDesiredVel(actor);
		vec3 e = leader->getPosition() - actorPos;
		vec3 vArrive = actor->KArrival * e;
		Vdesired = CSeparation * vSeparate + CArrival * vArrive;
	}
	// Linghan 2017-12-05

	return Vdesired;
}

// Chasing behavior
///////////////////////////////////////////////////////////////////////////////
// For the given actor, return a desired velocity vector in world coordinates
// In chasing mode, every one is trying to chase others within the neighborhood
// It works similar to Separation, but with an opposite direction of d
// Linghan 2017-12-05

Chasing::Chasing(vector<AActor>* agents)
{
	m_name = "separation";
	m_AgentList = agents;
}

Chasing::~Chasing()
{
}

Chasing::Chasing(Chasing& orig)
{
	m_name = "separation";
	m_AgentList = orig.m_AgentList;
}

vec3 Chasing::calcDesiredVel(BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);
	vec3 actorPos = actor->getPosition();
	vector<AActor>& agentList = *m_AgentList;

	// compute Vdesired = Vseparate
	vec3 gain = vec3(0.0, 0.0, 0.0);
	for (int i = 0; i < agentList.size(); i++) {
		BehaviorController* neigh_agent = agentList[i].getBehaviorController();
		if (neigh_agent == actor) continue;
		vec3 dist = neigh_agent->getPosition() - actorPos;
		if (dist.Length() <= actor->gKNeighborhood)
			gain = gain + 5 * dist / (dist.Length() * dist.Length());
		// assume all w_i are the same here -> 5
	}
	if (gain == vec3Zero)
		Vdesired = actor->getVelocity() / 10; // slow down
	else
		Vdesired = actor->KSeparation * gain; // assume KChasing = KSeparation

	return Vdesired;
}

// Snake behavior
///////////////////////////////////////////////////////////////////////////////
// For the given the actor, return a desired velocity in world coordinates
// Snake behavior makes the actor walks on a "S" path
// Linghan 2017-12-05


Snake::Snake()
{
	m_name = "snake";
	cnt = 0;
	xflag = false;
}

Snake::Snake(Snake& orig)
{
	m_name = "snake";
	cnt = 0;
	xflag = false;
}


Snake::~Snake()
{
}

vec3 Snake::calcDesiredVel(BehaviorController* actor)
{
	vec3 Vdesired = vec3(0.0, 0.0, 0.0);

	if(xflag)
		Vdesired[0] = 100;
	else
		Vdesired[2] = 100;

	if (cnt < 100) cnt++;
	if (cnt == 100) {
		cnt = 0; // reset
		xflag = !xflag;
	}
	return Vdesired;
}

///////////////////////////////////////////////////////////////////////////////


#include <SFML/Graphics.hpp>
#include <math.h>

class Convex : public sf::ConvexShape
{
public:
			Convex(sf::Vector2f velocity);											//Default constructor
	void	rotateAroundMassCenter(sf::Time*);										//Rotates each point of the convex around the mass center
	void	borderCollision(sf::RenderWindow*);										//Checking has any point of the convex hit a wall
	void	move(sf::Time*);														//Moves each point of the convex by velocity * dt
	void	setMassCenter();														//Calculates and returns mass center's current location
	void	getMassCenter(sf::Vector2f*);											//Sets the masscenter's position to the vector given as pointer
	void	drawCollisionPoint(sf::Vector2f*, sf::RenderWindow*);					//Applies the impulse to the convex's angular velocity and velocity
	void	iterate(sf::RenderWindow*, sf::Time*);									//Iterates angular velocity, velocity and position using the functions above
	float	getImpulse(sf::Vector2f*, sf::Vector2f*);								//Calculate the impulse between the convex and a wall represented by the collision normal
	float	getImpulse(sf::Vector2f*, sf::Vector2f*, Convex*, Convex*);				//Calculate the impulse between two convexes
	float	getRadius(int);															//Calculate radius between convex's point[i] and the convex's mass center
	void	applyChanges(int, sf::Vector2f*);										//Changes the velocity and angular velocity of the convex after collision
	void	applyChanges(sf::Vector2f*, float, sf::Vector2f*, Convex*, Convex*);	//Same as the above, but for multiple convexes
	float	vectorProduct(float x1, float y1, float x2, float y2);					//Returns the vector product of the parameters (x1 * y2 - y1 * x2)
	void	convexCollision(Convex*, sf::RenderWindow*);							//Calculates the coordinates of the collision point
	bool	isBetween(float, float, float);											//Checks if the first parameter's  value is between the values of the two other parameters
	bool	hasCollided(Convex*);													//Checks has this convex collided with the parameter convex
private:
	float			_mass;				//kg
	float			_angularVelocity;	//rad/s
	float			_momentOfInertia;	//kg*m^2
	sf::Vector2f	_massCenter;		//x,y
	sf::Vector2f	_velocity;			//vx, vy
};
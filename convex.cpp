#include "convex.h"
#include <iostream>

//Default constructor
Convex::Convex(sf::Vector2f velocity) : sf::ConvexShape()
{
	_mass = 5;
	_velocity = velocity;
	_angularVelocity = 2;
	_momentOfInertia = _mass;
}

//Move each point by velocity * dt
void Convex::move(sf::Time* dt)
{
	for (int i = 0; i < getPointCount(); i++)
	{
		setPoint(i, getPoint(i) + _velocity * dt->asSeconds());
	}
}

/*
	Rotates every point of the convex around its center of mass
*/
void Convex::rotateAroundMassCenter(sf::Time* dt)
{
	sf::Vector2f temp;

	for (int i = 0; i < getPointCount(); i++)
	{
		temp = getPoint(i);
		temp -= _massCenter;
		temp.x = (getPoint(i).x - _massCenter.x) * cos(_angularVelocity * dt->asSeconds()) - (getPoint(i).y - _massCenter.y) * sin(_angularVelocity * dt->asSeconds());
		temp.y = (getPoint(i).x - _massCenter.x) * sin(_angularVelocity * dt->asSeconds()) + (getPoint(i).y - _massCenter.y) * cos(_angularVelocity * dt->asSeconds());
		temp += _massCenter;
		setPoint(i, temp);
	}
}

/*
	Calculates the center of mass' components
	X1+X2+...Xn / amount of points
	Y1+Y2+...Yn / amount of points
*/
void Convex::setMassCenter()
{
	sf::Vector2f temp(0,0);

	for (int i = 0; i < getPointCount(); i++)
	{
		temp.x += getPoint(i).x;
		temp.y += getPoint(i).y;
	}

	temp.x = temp.x / getPointCount();
	temp.y = temp.y / getPointCount();

	_massCenter = temp;
}

void Convex::applyChanges(int index, sf::Vector2f* collisionNormal)
{
	float radius = getRadius(index);

	sf::Vector2f collisionPoint(getPoint(index).x - _massCenter.x,
		getPoint(index).y - _massCenter.y);

	_momentOfInertia = _mass * pow(radius, 2);

	float impulse = getImpulse(&collisionPoint, collisionNormal);

	_velocity.x += impulse / _mass * collisionNormal->x;
	_velocity.y += impulse / _mass * collisionNormal->y;

	_angularVelocity = _angularVelocity + impulse / _momentOfInertia * (collisionPoint.x * collisionNormal->y - collisionPoint.y * collisionNormal->x);
}

/*
	Applies the changes caused by the collision to both collided convexes
*/
void Convex::applyChanges(sf::Vector2f* collisionNormal, float impulse, sf::Vector2f* collisionPoint, Convex* shapeA, Convex* shapeB)
{
	shapeA->setMassCenter();
	shapeB->setMassCenter();

	sf::Vector2f radiusA(collisionPoint->x - shapeA->_massCenter.x,
						 collisionPoint->y - shapeA->_massCenter.y);
	sf::Vector2f radiusB(collisionPoint->x - shapeB->_massCenter.x,
						 collisionPoint->y - shapeB->_massCenter.y);
	
	float radiusAlength = sqrt(pow(radiusA.x, 2) + pow(radiusA.y, 2));
	float radiusBlength = sqrt(pow(radiusB.x, 2) + pow(radiusB.y, 2));


	//float Ja = _mass * pow(radiusAlength, 2);
	//float Jb = shapeB->_mass * pow(radiusBlength, 2);

	float Ja = 44;
	float Jb = 45;

	shapeA->_velocity.x += impulse / shapeA->_mass * collisionNormal->x;
	shapeA->_velocity.y += impulse / shapeA->_mass * collisionNormal->y;

	shapeB->_velocity.x -= impulse / shapeB->_mass * collisionNormal->x;
	shapeB->_velocity.y -= impulse / shapeB->_mass * collisionNormal->y;



	shapeA->_angularVelocity += impulse / Ja * vectorProduct(radiusA.x,
															 radiusA.y,
															 collisionNormal->x,
															 collisionNormal->y);

	shapeB->_angularVelocity -= impulse / Ja * vectorProduct(radiusB.x,
															  radiusB.y,
															  collisionNormal->x,
															  collisionNormal->y);
}

/*
	Checks the position of every point of the vertex, if
	any of them are outside of the window and still heading
	there changes their direction

	momentOfInertia = mass * r^2
*/
void Convex::borderCollision(sf::RenderWindow* window)
{
	float impulse;
	float Px, Py;

	for (int i = 0; i < getPointCount(); i++)
	{
		
		if (getPoint(i).x < 0 && _velocity.x < 0)
		{
			sf::Vector2f collisionNormal(1, 0);
			applyChanges(i, &collisionNormal);
		}

		if (getPoint(i).x > window->getSize().x && _velocity.x > 0)
		{
			sf::Vector2f collisionNormal(-1, 0);
			applyChanges(i, &collisionNormal);
		}

		if (getPoint(i).y < 0 && _velocity.y < 0)
		{
			sf::Vector2f collisionNormal(0, 1);
			applyChanges(i, &collisionNormal);
		}

		if (getPoint(i).y > window->getSize().y && _velocity.y > 0)
		{
			sf::Vector2f collisionNormal(0, -1);
			applyChanges(i, &collisionNormal);
		}

		_momentOfInertia = _mass;
	}
}

/*
	Calculates the distance between point[i]
	and the convex's center of mass
*/
float Convex::getRadius(int pointIndex)
{
	float radius = sqrt(pow(getPoint(pointIndex).x - getPosition().x, 2) + 
						pow(getPoint(pointIndex).y - getPosition().y, 2));
	return radius;
}

/*
	Calculates the impulse between the convex and a border represented by the collisionNormal
*/
float Convex::getImpulse(sf::Vector2f* collisionPoint, sf::Vector2f* collisionNormal)
{
	float impulse = -(1 + 0.5) * ((_velocity.x - _angularVelocity * collisionPoint->y) * collisionNormal->x +
		(_velocity.y + _angularVelocity * collisionPoint->x) * collisionNormal->y) /
		((1 / _mass) + (pow(collisionPoint->x * collisionNormal->y - collisionPoint->y * collisionNormal->x, 2)) / _momentOfInertia);

	return impulse;
}

/*
	Calculates the impulse between two convexes
*/
float Convex::getImpulse(sf::Vector2f* collisionPoint, sf::Vector2f* collisionNormal, Convex* shapeA, Convex* shapeB)
{
	shapeA->setMassCenter();
	shapeB->setMassCenter();

	sf::Vector2f radiusAP(collisionPoint->x - shapeA->_massCenter.x,
						  collisionPoint->y - shapeA->_massCenter.y);
	sf::Vector2f radiusBP(collisionPoint->x - shapeB->_massCenter.x,
						  collisionPoint->y - shapeB->_massCenter.y);

	float radiusAlength = sqrt(pow(radiusAP.x, 2) + pow(radiusAP.y, 2));
	float radiusBlength = sqrt(pow(radiusBP.x, 2) + pow(radiusBP.y, 2));

	float Ja = 44;
	float Jb = 45;

	sf::Vector2f angularVelocityByRadiusAP(-(shapeA->_angularVelocity * radiusAP.y), -(-(shapeA->_angularVelocity * radiusAP.x)));
	sf::Vector2f angularVelocityByRadiusBP(-(shapeB->_angularVelocity * radiusBP.y), -(-(shapeB->_angularVelocity * radiusBP.x)));
	sf::Vector2f Vap(shapeA->_velocity.x + angularVelocityByRadiusAP.x, shapeA->_velocity.y + angularVelocityByRadiusAP.y);
	sf::Vector2f Vab(shapeB->_velocity.x + angularVelocityByRadiusBP.x, shapeB->_velocity.y + angularVelocityByRadiusBP.y);

	sf::Vector2f relativeSpeed = Vap - Vab;

	float impulse = -(1 + 0.5) *  ((shapeA->_velocity.x - shapeA->_angularVelocity * radiusAP.y - shapeB->_velocity.x + shapeB->_angularVelocity * radiusBP.y) * collisionNormal->x +
		(shapeA->_velocity.y + shapeA->_angularVelocity * radiusAP.x - shapeB->_velocity.y - shapeB->_angularVelocity * radiusBP.x) * collisionNormal->y)
		/
		((1 / shapeA->_mass) + (1 / shapeB->_mass) + (pow(radiusAP.x * collisionNormal->y - radiusAP.y * collisionNormal->x, 2) / Ja)
			+ (pow(radiusBP.x * collisionNormal->y - radiusBP.y * collisionNormal->x, 2) / Jb));


	if (impulse < 0)
	{
		collisionNormal->x *= -1;
		collisionNormal->y *= -1;

		float impulse = -(1 + 0.5) *  ((shapeA->_velocity.x - shapeA->_angularVelocity * radiusAP.y - shapeB->_velocity.x + shapeB->_angularVelocity * radiusBP.y) * collisionNormal->x +
			(shapeA->_velocity.y + shapeA->_angularVelocity * radiusAP.x - shapeB->_velocity.y - shapeB->_angularVelocity * radiusBP.x) * collisionNormal->y)
			/
			((1 / shapeA->_mass) + (1 / shapeB->_mass) + (pow(radiusAP.x * collisionNormal->y - radiusAP.y * collisionNormal->x, 2) / Ja)
				+ (pow(radiusBP.x * collisionNormal->y - radiusBP.y * collisionNormal->x, 2) / Jb));
	}

	if (impulse < 0)
	{
		std::cout << collisionNormal->x << ", " << collisionNormal->y << std::endl;
	}
	
	return impulse;
}

void Convex::drawCollisionPoint(sf::Vector2f* point, sf::RenderWindow* window)
{
	sf::CircleShape circle(2);
	circle.setPosition(point->x, point->y);
	window->draw(circle);
}

float Convex::vectorProduct(float x1, float y1, float x2, float y2)
{
	float vectorProduct = x1 * y2 - y1 * x2;

	return vectorProduct;
}

/*
	Checks if the 'value' is between startPoint and endPoint
*/
bool Convex::isBetween(float value, float startPoint, float endPoint)
{
	if (value < startPoint && value > endPoint)
	{
		return true;
	}
	if (value > startPoint && value < endPoint)
	{
		return true;
	}
	return false;
}

bool Convex::hasCollided(Convex* other)
{
	sf::Vector2f collisionPoint, betweenCorners;


	for (int i = 0; i < other->getPointCount(); i++)
	{
		for (int j = 0; j < getPointCount(); j++)
		{
			if (j < getPointCount() - 1)
			{
				sf::Vector2f collisionPoint(other->getPoint(i).x - getPoint(j).x,
					other->getPoint(i).y - getPoint(j).y);
				sf::Vector2f betweenCorners(getPoint(j + 1).x - getPoint(j).x,
					getPoint(j + 1).y - getPoint(j).y);
			}
			else
			{
				sf::Vector2f collisionPoint(other->getPoint(i).x - getPoint(j).x,
					other->getPoint(i).y - getPoint(j).y);
				sf::Vector2f betweenCorners(getPoint(0).x - getPoint(j).x,
					getPoint(0).y - getPoint(j).y);
			}

			if (vectorProduct(betweenCorners.x, betweenCorners.y, collisionPoint.x, collisionPoint.y) > 0)
			{
				return false;
			}
		}
	}

	for (int i = 0; i < this->getPointCount(); i++)
	{
		for (int j = 0; j < other->getPointCount(); j++)
		{
			if (j < other->getPointCount() - 1)
			{
				sf::Vector2f collisionPoint(this->getPoint(i).x - other->getPoint(j).x,
					this->getPoint(i).y - other->getPoint(j).y);
				sf::Vector2f betweenCorners(other->getPoint(j + 1).x - other->getPoint(j).x,
					other->getPoint(j + 1).y - other->getPoint(j).y);
			}
			else
			{
				sf::Vector2f collisionPoint(this->getPoint(i).x - other->getPoint(j).x,
					this->getPoint(i).y - other->getPoint(j).y);
				sf::Vector2f betweenCorners(other->getPoint(0).x -other->getPoint(j).x,
					other->getPoint(0).y - other->getPoint(j).y);
			}

			if (vectorProduct(betweenCorners.x, betweenCorners.y, collisionPoint.x, collisionPoint.y) > 0)
			{
				return false;
			}
		}
	}
	return true;
}

/*
	Calculates the coordinates of the collision point
*/
void Convex::convexCollision(Convex* other, sf::RenderWindow* window)
{
	sf::Vector2f collisionNormal;
	float collisionX, collisionY;
	float x1, x2, x3, x4, y1, y2, y3, y4;
	

	if (hasCollided(other))
	{
		for (int i = 0; i < getPointCount(); i++)
		{
			//Retrieves the coordinates of the point i
			x1 = this->getPoint(i).x;
			y1 = this->getPoint(i).y;

			//If the index 'i' equals the amount of points
			//the other end of the line should be the starting point
			if (i == getPointCount() - 1)
			{
				x2 = this->getPoint(0).x;
				y2 = this->getPoint(0).y;
			}
			else
			{
				x2 = this->getPoint(i + 1).x;
				y2 = this->getPoint(i + 1).y;
			}


			for (int j = 0; j < other->getPointCount(); j++)
			{
				x3 = other->getPoint(j).x;
				y3 = other->getPoint(j).y;

				//If the index 'j' equals the amount of points
				//the other end of the line should be the starting point
				if (i == getPointCount() - 1)
				{
					x4 = other->getPoint(0).x;
					y4 = other->getPoint(0).y;
				}
				else
				{
					x4 = other->getPoint(i + 1).x;
					y4 = other->getPoint(i + 1).y;
				}

				collisionX = vectorProduct(vectorProduct(x1, y1, x2, y2),
							 vectorProduct(x1, 1, x2, 1),
							 vectorProduct(x3, y3, x4, y4),
							 vectorProduct(x3, 1, x4, 1))
							 /
							 vectorProduct(vectorProduct(x1, 1, x2, 1),
							 vectorProduct(y1, 1, y2, 1),
							 vectorProduct(x3, 1, x4, 1),
							 vectorProduct(y3, 1, y4, 1));

				collisionY = vectorProduct(vectorProduct(x1, y1, x2, y2),
							 vectorProduct(y1, 1, y2, 1),
							 vectorProduct(x3, y3, x4, y4),
							 vectorProduct(y3, 1, y4, 1))
							 /
							 vectorProduct(vectorProduct(x1, 1, x2, 1),
							 vectorProduct(y1, 1, y2, 1),
							 vectorProduct(x3, 1, x4, 1),
							 vectorProduct(y3, 1, y4, 1));
				
				//Checking that the collision point is actually between the corners
				//of the convexes
				if (isBetween(collisionX, x1, x2) && isBetween(collisionX, x3, x4) &&
					isBetween(collisionY, y1, y2) && isBetween(collisionY, y3, y4))
				{
					//If the collision point is between these points use them to calculate the
					//collisionNormal's components
					if (isBetween(collisionX, x1, x2) && isBetween(collisionY, y1, y2))
					{
						float x = x2 - x1;
						float y = y2 - y1;
						
						float length = sqrt(pow(x, 2) + pow(y, 2));
						float unitX = x / length;
						float unitY = y / length;

						collisionNormal.x = -unitY;
						collisionNormal.y = unitX;

						float impulse = getImpulse(&sf::Vector2f(collisionX, collisionY), &collisionNormal, other, this);
						applyChanges(&collisionNormal, impulse, &sf::Vector2f(collisionX, collisionY), other, this);
					}
					else
					{
						float x = x4 - x3;
						float y = y4 - y3;

						float length = sqrt(pow(x, 2) + pow(y, 2));
						float unitX = x / length;
						float unitY = y / length;

						collisionNormal.x = -unitY;
						collisionNormal.y = unitX;

						float impulse = getImpulse(&sf::Vector2f(collisionX, collisionY), &collisionNormal, this, other);
						applyChanges(&collisionNormal, impulse, &sf::Vector2f(collisionX, collisionY), this, other);
					}
					
					sf::CircleShape circle(5);
					circle.setFillColor(sf::Color::Red);
					circle.setPosition(collisionX, collisionY);
					window->draw(circle);
				}
			}
		}
	}
}

/*
	Iterates the position of each point by velocity, and angular velocity.
	In addition checks whether or not the convex has collided with the borders
	of the window, also handles the drawing of the object.
*/
void Convex::iterate(sf::RenderWindow* window, sf::Time* dt)
{
	window->draw(*this);
	rotateAroundMassCenter(dt);
	borderCollision(window);
	setMassCenter();
	move(dt);
}
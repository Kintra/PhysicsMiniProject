#include <vector>
#include "convex.h"
#include "definitions.h"
#include <iostream>
#include <time.h>
using namespace std;

int main()
{
	//Initializing random number genetaror
	srand(time(0));

	//Stores the help lines which display
	//the edges of the current object
	sf::VertexArray lines(sf::LinesStrip);

	// create an empty shape
	Convex*	convex;

	//Creating the window, resolution and window title as parameters
	sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), WINDOW_TITLE);

	//Used to keep track of passed time
	sf::Clock deltaClock;

	//Stores pointers to the ConvexShapes
	vector<Convex*> ConvexShapes;

	//Stores the points of a single convex until it's creation
	vector<sf::Vector2f> ClickLocations;

	//Updating the contents of the window
	while (window.isOpen())
	{
		//Start each loop by reseting deltatime
		sf::Time dt = deltaClock.restart();

		//Clear the window of all drawn objects
		window.clear();

		//Checking events
		sf::Event event;

		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
				window.close();

			//Checking has mouse button been pressed
			if (event.type == sf::Event::MouseButtonPressed)
			{
				//If the pressed mouse button is the left one start drawing the "help lines"
				//in addition to storing the click positions to ClickLocations list
				if (event.mouseButton.button == sf::Mouse::Left)
				{
					//Saving the cursor's current location in the ClickLocations list's last index
					ClickLocations.push_back(static_cast<sf::Vector2f>(sf::Mouse::getPosition(window)));

					//Resizing the lines array every time left mouse button is pressed
					lines.resize(ClickLocations.size() + 1);
				}

				//If right mouse button is pressed and there are convex shapes in the ConvexShapes list
				//remove one of them
				if (event.mouseButton.button == sf::Mouse::Right)
				{
					if (ConvexShapes.size() > 0)
					{
						ConvexShapes.pop_back();
					}
				}
			}

			//Checking if any key is pressed
			if (event.type == sf::Event::KeyPressed)
			{
				//If the pressed key is Escape stop drawing
				if (event.key.code == sf::Keyboard::Z)
				{
					//Create new Convex
					//convex = new Convex(sf::Vector2f(sf::Vector2f(rand() % 5, rand() % 5)));
					convex = new Convex(sf::Vector2f(sf::Vector2f(-5, 10)));

					//Resize the convex to fit all of ClickLocation list's contents
					convex->setPointCount(ClickLocations.size());

					//Set the clicked points as points of the convex
					for (int i = 0; i < ClickLocations.size(); i++)
					{
						convex->setPoint(i, ClickLocations[i]);
					}
					//Set a fill color for the convex
					convex->setFillColor(sf::Color(0, 255, 0, 200));

					//Add the location of the convex to convexShapes list
					ConvexShapes.push_back(convex);
					//Clear the list of click locations so future shapes
					//won't have the same points
					ClickLocations.clear();
					//Clear the "help lines"
					lines.clear();
					//Making sure the pointer no longer points to the convex
					convex = 0;
				}
			}
		}

		//Draw the "help lines" on the screen between the
		//click locations, lines > ClickLocations so the last
		//point will be the cursor's current position
		for (int i = 0; i < ClickLocations.size(); i++)
		{
			lines[i].position = ClickLocations[i];
		}

		//Set the cursor's current location as the last point only if the list
		//already consists of atleast another point
		if (lines.getVertexCount() > 0)
		{
			lines[ClickLocations.size()].position = static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
		}

		//Moving every Convex on the ConvexShapes list
		for (int i = 0; i < ConvexShapes.size(); i++)
		{
			window.draw(*ConvexShapes[i]);
			ConvexShapes[i]->iterate(&window, &dt);

			for (int j = i + 1; j < ConvexShapes.size(); j++) {
				ConvexShapes[i]->convexCollision(ConvexShapes[j], &window);
			}
		}

		//Draws the contents of the lines vertexarray
		window.draw(lines);

		//Display every drawnobject 
		window.display();
	}

	return 0;
}
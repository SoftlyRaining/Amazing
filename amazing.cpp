/*
* Things to study:
* graphs
* - find articulation vertices
* - topological sort DAGs
* - strongly connected components (directed)
* - path finding
* weighted graphs
* - pathing between nodes
* - dijkstra
* - A* search, bidirectional...
* n-choose-k type problems
* dynamic programming
* bloom filter
*
*
* bonus:
* constraint satisfaction?
* monte carlo?
*/

#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <concepts>
#include <fstream>
#include <format>
#include <functional>
#include <iostream>
#include <iterator>
#include <list>
#include <stdexcept>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <vector>
#include <set>

#include <SDL.h>

class SDLContext {
public:
	SDLContext(int width, int height, int pixelSize) : width(width), height(height), pixelSize(pixelSize) {
		if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0)
			throw "couldn't init SDL";

		if (!SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0"))
			std::cerr << "warning: pixelated texture filtering not enabled\n";
		SDLWindow = SDL_CreateWindow("Maze", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width * pixelSize, height * pixelSize, SDL_WINDOW_SHOWN);
		if (SDLWindow == NULL) {
			throw "couldn't create window";
		}
		SDLRenderer = SDL_CreateRenderer(SDLWindow, -1, SDL_RENDERER_ACCELERATED);
		SDL_RenderSetLogicalSize(SDLRenderer, width, height);
		SDL_SetRenderDrawBlendMode(SDLRenderer, SDL_BLENDMODE_BLEND);
	}

	~SDLContext() {
		SDL_DestroyRenderer(SDLRenderer);
		SDL_DestroyWindow(SDLWindow);

		SDL_Quit();
	}

	SDL_Renderer* renderer() const {
		return SDLRenderer;
	}

public:
	const int width;
	const int height;
	const int pixelSize;

private:
	SDL_Window* SDLWindow;
	SDL_Renderer* SDLRenderer;
};

enum class VerticalDirection {
	down = -1,
	flat = 0,
	up = 1
};

enum class TraversalState {
	undiscovered,
	discovered,
	processed
};

class Cell {
public:
	int x{}, y{}, z{};
	bool open{};
	std::bitset<4> connections{};
	std::array<VerticalDirection, 4> verticalConnections{ VerticalDirection::flat, VerticalDirection::flat, VerticalDirection::flat, VerticalDirection::flat };
	TraversalState state{ TraversalState::undiscovered };
};

class Maze {
public:
	static constexpr int pixelSize = 2;
	static constexpr int cellSize = 16;

	Maze(int screenWidth, int screenHeight) :
		cellWidth(screenWidth / pixelSize / cellSize),
		cellHeight(screenHeight / pixelSize / cellSize)
	{
		// round down to full cell size
		screenWidth /= pixelSize;
		screenHeight /= pixelSize;
		screenWidth -= screenWidth % cellSize;
		screenHeight -= screenHeight % cellSize;

		context = std::make_unique<SDLContext>(screenWidth, screenHeight, pixelSize);
		initTextures();

		// initialize maze grid
		cells.resize(cellWidth * cellHeight * layers);
		for (int z = 0; z < layers; z++) {
			for (int y = 0; y < cellHeight; y++) {
				for (int x = 0; x < cellWidth; x++) {
					Cell* c = getCell(x, y, z);
					c->x = x;
					c->y = y;
					c->z = z;
				}
			}
		}

		// initial (blank) render
		//SDL_SetRenderDrawColor(context->renderer(), 0x88, 0x88, 0x88, 0xff);
		//SDL_RenderFillRect(context->renderer(), NULL);
		for (int i = 0; i < cells.size(); i++) {
			int x = i % cellWidth;
			int y = i / cellWidth;
			SDL_Rect destRect = { x * cellSize, y * cellSize, cellSize, cellSize };
			SDL_RenderCopy(context->renderer(), tileTextures[0], NULL, &destRect);
		}
		SDL_RenderPresent(context->renderer());
	}

	void generate(const double branchChance, const double loopChance, const double bridgeChance) {
		int startX = 5 + rand() % (width() - 10); // not too close to edges (increases chance that graph will not end too early)
		int startY = 5 + rand() % (height() - 10);
		Cell* start = getCell(startX, startY, 0);

		std::vector<Cell*> threads;
		start->open = true;
		threads.push_back(start); // start in two directions from this point
		threads.push_back(start);

		while (!threads.empty()) {
			Cell* c = threads.front();
			threads.erase(threads.begin());
			do {
				int offset = rand() % 4;
				int i = 0;
				for (; i < 4; i++) {
					int direction = (i + offset) % 4;
					if (c->connections[direction])
						continue; // already connected that way
					// try to make a connection in that direction
					Cell* neighbor = getNeighbor(c, direction);
					if (neighbor == NULL)
						continue;
					bool looping = neighbor->open;
					bool canBridgeOver = false;
					if (looping) {
						Cell* otherSideOfNeighbor = getNeighbor(neighbor, direction);
						canBridgeOver = otherSideOfNeighbor != NULL && !otherSideOfNeighbor->open
							&& !neighbor->connections[direction]
							&& neighbor->connections[(direction + 1) % 4]
							&& neighbor->connections[(direction + 3) % 4];
						if (canBridgeOver && ((double)rand() / RAND_MAX) < bridgeChance) {
							// do a bridge
							neighbor = getCell(neighbor->x, neighbor->y, neighbor->z + 1); // layer above

							c->connections[direction] = true;
							c->verticalConnections[direction] = VerticalDirection::up;
							neighbor->connections[(direction + 2) % 4] = true;
							neighbor->verticalConnections[(direction + 2) % 4] = VerticalDirection::down;
							neighbor->open = true;

							neighbor->connections[direction] = true;
							neighbor->verticalConnections[direction] = VerticalDirection::down;
							otherSideOfNeighbor->connections[(direction + 2) % 4] = true;
							otherSideOfNeighbor->verticalConnections[(direction + 2) % 4] = VerticalDirection::up;
							otherSideOfNeighbor->open = true;

							renderCell(c);
							renderCell(neighbor);
							renderCell(otherSideOfNeighbor);
							present();

							threads.push_back(otherSideOfNeighbor);
							break;
						}
					}
					if (looping && ((double)rand() / RAND_MAX) >= loopChance)
						continue;

					c->connections[direction] = true;
					neighbor->connections[(direction + 2) % 4] = true;
					neighbor->open = true;

					renderCell(c);
					renderCell(neighbor);
					present();

					// don't continue if we're looping into existing structure - nowhere to go
					if (!looping)
						threads.push_back(neighbor); 
					break;
				}
				if (i == 4)
					break; // dead end - don't consider branching further
			} while (((double)rand() / RAND_MAX) < branchChance);
		}

		// pick out a start and end point - try to place them at network diameter
		// that is, the longest shortest path between nodes
		Cell* farthestCell = start;
		std::function<void(Cell*, Cell*)> nopEdge = [&](Cell* p, Cell* c) -> void {};
		std::function<void(Cell*)> nopVertex = [&](Cell* c) -> void {};
		std::function<void(Cell*)> lateVertex = [&](Cell* c) -> void { farthestCell = c; };
		BFS(start, nopVertex, lateVertex, nopEdge);

		std::vector<Cell*> prevLinks(size(), NULL);
		auto getIndex = [&](Cell* c) -> size_t { return c - data(); };
		std::function<void(Cell*, Cell*)> prevLinkEdge = [&](Cell* p, Cell* c) -> void {
			if (c->state == TraversalState::undiscovered)
				prevLinks[getIndex(c)] = p;
		};
		BFS(farthestCell, nopVertex, lateVertex, prevLinkEdge);

		while (farthestCell != NULL) {
			solution.push_back(farthestCell);
			farthestCell = prevLinks[getIndex(farthestCell)];
		};

		if (solution.empty())
			throw "no solution?";
		SDL_Rect destRect = { solution[0]->x * cellSize, solution[0]->y * cellSize, cellSize, cellSize };
		SDL_RenderCopy(context->renderer(), startTex, NULL, &destRect);
		destRect.x = solution[solution.size()-1]->x * cellSize;
		destRect.y = solution[solution.size() - 1]->y * cellSize;
		SDL_RenderCopy(context->renderer(), endTex, NULL, &destRect);
		present();
	}

	void BFS(Cell* startPoint, std::function<void(Cell*)> earlyVertex, std::function<void(Cell*)> lateVertex, std::function<void(Cell*, Cell*)> edge) {
		resetTraversalState();

		if (startPoint == NULL) {
			// find our own arbitrary start point
			for (Cell& c : cells) {
				if (c.open) {
					startPoint = &c;
					break;
				}
			}
			throw "no open cells to start search";
		}

		std::vector<Cell*> threads;
		threads.push_back(startPoint);
		startPoint->state = TraversalState::discovered;

		while (!threads.empty()) {
			Cell* c = threads.front();
			threads.erase(threads.begin());
			earlyVertex(c);

			for (int direction = 0; direction < 4; direction++) {
				if (!c->connections[direction])
					continue;
				Cell* n = getNeighbor(c, direction, c->verticalConnections[direction]);
				if (n == NULL)
					throw "followed bad connection";

				edge(c, n);
				if (n->state == TraversalState::undiscovered) {
					n->state = TraversalState::discovered;
					threads.push_back(n);
				}
			}
			c->state = TraversalState::processed;
			lateVertex(c);
		}
	}

	Cell* getCell(int x, int y, int layer) {
		if (x < 0 || y < 0 || layer < 0 || x >= cellWidth || y >= cellHeight || layer >= layers)
			return NULL;
		return &cells[x + cellWidth * y + cellWidth*cellHeight*layer];
	}
	Cell* getNeighbor(Cell* c, int direction, VerticalDirection verticalDirection = VerticalDirection::flat) {
		switch (direction) {
		case 0: // right
			return getCell(c->x + 1, c->y, c->z + static_cast<int>(verticalDirection));
		case 1: // up
			return getCell(c->x, c->y - 1, c->z + static_cast<int>(verticalDirection));
		case 2: // left
			return getCell(c->x - 1, c->y, c->z + static_cast<int>(verticalDirection));
		case 3: //  down
			return getCell(c->x, c->y + 1, c->z + static_cast<int>(verticalDirection));
		default:
			throw "unhandled direction";
		}
	}

	void resetTraversalState() {
		for (Cell& c : cells) {
			c.state = TraversalState::undiscovered;
		}
	}

	void renderCell(Cell* c) {
		size_t textureIndex = c->connections.to_ulong();
		SDL_Rect destRect = { c->x * cellSize, c->y * cellSize, cellSize, cellSize };
		SDL_RenderCopy(context->renderer(), tileTextures[textureIndex], NULL, &destRect);

		if (solution.empty())
			return;
		if (c == solution[0]) {
			SDL_Rect destRect = { c->x * cellSize, c->y * cellSize, cellSize, cellSize };
			SDL_RenderCopy(context->renderer(), startTex, NULL, &destRect);
		}
		else if (c == solution[solution.size() - 1]) {
			SDL_Rect destRect = { c->x * cellSize, c->y * cellSize, cellSize, cellSize };
			SDL_RenderCopy(context->renderer(), endTex, NULL, &destRect);
		}
	};
	void renderPath(std::vector<Cell*>& path, const Uint32 color) {
		SDL_SetRenderDrawColor(context->renderer(), color >> 24, (color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff);

		auto drawConnection = [this](Cell* c, int direction) -> void {
			// don't draw if covered by another cell
			Cell* above = getCell(c->x, c->y, c->z + 1);
			if (above != NULL && above->open)
				return;

			bool isHorizontal = direction % 2 == 0;
			SDL_Rect rect = {
				c->x * cellSize + (direction==2 ? 0 : 3),
				c->y * cellSize + (direction==1 ? 0 : 3),
				cellSize - (isHorizontal ? 3 : 6),
				cellSize - (!isHorizontal ? 3 : 6)
			};
			SDL_RenderFillRect(context->renderer(), &rect);
		};

		for (int i = 1; i < path.size(); i++) {
			int dx = path[i]->x - path[i - 1ll]->x;
			int dy = path[i]->y - path[i - 1ll]->y;
			
			int direction = 0;
			if (dx != 0)
				direction = (dx > 0) ? 0 : 2;
			else if (dy != 0)
				direction = (dy > 0) ? 3 : 1;
			else
				throw "path doesn't make sense";

			drawConnection(path[i], (direction + 2) % 4);
			drawConnection(path[i - 1ll], direction);
		}
	}
	void renderThinPath(std::vector<Cell*>& path, const Uint32 color) {
		SDL_SetRenderDrawColor(context->renderer(), color >> 24, (color >> 16) & 0xff, (color >> 8) & 0xff, color & 0xff);

		const int pathCount = (cellSize - 6) / 2;
		static int counter = -1;
		counter++;
		int pathIndex = counter % pathCount;
		int offset = 3 + pathIndex * 2;

		for (int i = 1; i < path.size(); i++) {
			SDL_RenderDrawLine(
				context->renderer(),
				path[i - 1ll]->x * cellSize + offset,
				path[i - 1ll]->y * cellSize + offset,
				path[i]->x * cellSize + offset,
				path[i]->y * cellSize + offset
			);
		}
	}
	void clearCell(Cell* c) {
		renderCell(c);
		rerenderCellsAbove(c);
	}
	void clearPath(std::vector<Cell*>& path) {
		for (Cell* c : path)
			clearCell(c);
	}
	void present() { SDL_RenderPresent(context->renderer()); }

	size_t width() { return cellWidth; }
	size_t height() { return cellHeight; }
	size_t size() { return cells.size(); }

	Cell* data() { return cells.data(); }
	Cell* getStart() { return solution.empty() ? NULL : solution[0]; }
	Cell* getFinish() { return solution.empty() ? NULL : solution[solution.size()-1]; }

private:
	void initTextures() {
		// set up textures
		std::array<SDL_Surface*, 1 << 4> tileSurfaces;

		constexpr Uint32 rmask = 0xff000000, gmask = 0x00ff0000, bmask = 0x0000ff00, amask = 0x000000ff;
		auto makeSurf = [&]() -> SDL_Surface* {
			SDL_Surface* surface = SDL_CreateRGBSurface(0, cellSize, cellSize, 32, rmask, gmask, bmask, amask);
			SDL_SetSurfaceBlendMode(surface, SDL_BLENDMODE_NONE);
			SDL_FillRect(surface, NULL, 0x00000000); // transparent
			return surface;
		};
		auto makeTex = [&](SDL_Surface* surf) -> SDL_Texture* {
			SDL_Texture* texture = SDL_CreateTextureFromSurface(context->renderer(), surf);
			if (texture == NULL)
				throw "unable to create texture";
			SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_BLEND);
			return texture;
		};

		{
			SDL_Surface* endSurf = makeSurf();
			Uint32* data = (Uint32*)endSurf->pixels;
			for (int i = 1; i <= cellSize / 2 - 3; i++) {
				for (int j = -i; j < i; j++) {
					int x = cellSize / 2 + j;
					data[x + cellSize * (i + 2)] = 0x000000ff;
					data[x + cellSize * (cellSize - 3 - i)] = 0x000000ff;
				}
			}
			endTex = makeTex(endSurf);
		}
		{
			SDL_Surface* startSurf = makeSurf();
			SDL_Rect endRect = { 3, 3, cellSize - 6, cellSize - 6 };
			SDL_FillRect(startSurf, &endRect, 0x000000ff);
			startTex = makeTex(startSurf);
		}

		// empty tile texture at index 0
		tileSurfaces[0] = makeSurf();
		constexpr Uint32 colors[] = { 0x000000ff, 0xffffffff };
		Uint32* data = (Uint32*)tileSurfaces[0]->pixels;
		for (int y = 0; y < cellSize; y++)
			for (int x = 0; x < cellSize; x++)
				data[y * cellSize + x] = colors[(x + y) % 2];

		// maze tiles
		for (int i = 1; i < 1 << 4; i++) {
			SDL_Surface* surface = makeSurf();
			tileSurfaces[i] = surface;

			bool right = i & 1;
			bool up = i & 2;
			bool left = i & 4;
			bool down = i & 8;

			Uint32 color = 0x000000ff;
			for (int margin = 1; margin <= 2; margin++) {
				// horizontal connections
				int longthMargin = 2 * margin;
				if (right)
					longthMargin -= margin;
				if (left)
					longthMargin -= margin;
				SDL_Rect rect = {
					left ? 0 : margin,
					margin,
					cellSize - longthMargin,
					cellSize - 2 * margin };
				SDL_FillRect(surface, &rect, color);

				// vertical connections
				longthMargin = 2 * margin;
				if (up)
					longthMargin -= margin;
				if (down)
					longthMargin -= margin;
				rect = {
					margin,
					up ? 0 : margin,
					cellSize - 2 * margin,
					cellSize - longthMargin };
				SDL_FillRect(surface, &rect, color);

				color = 0xffffffff; // switch to white for second time around
			}
		}

		for (int i = 0; i < 1 << 4; i++)
			tileTextures[i] = makeTex(tileSurfaces[i]);
	}

	void rerenderCellsAbove(Cell* c) {
		for (int z = c->z + 1; z < layers; z++) {
			Cell* zCell = getCell(c->x, c->y, z);
			if (zCell->open)
				renderCell(zCell);
		}
	}

private:
	std::unique_ptr<SDLContext> context;

	// textures
	std::array<SDL_Texture*, 1 << 4> tileTextures;
	SDL_Texture* startTex;
	SDL_Texture* endTex;

	// maze data
	static constexpr size_t layers = 2;
	size_t cellWidth, cellHeight;
	std::vector<Cell> cells;

	std::vector<Cell*> solution;
};

int main(int argc, char* args[]) {
	srand(static_cast<unsigned int>(time(NULL)));
	bool running = true;

	auto waitKeyCheckQuit = [&]() -> SDL_Keycode {
		SDL_Event e;
		do {
			SDL_WaitEvent(&e);
			if (e.type == SDL_QUIT)
				running = false;
			else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)
				running = false;
		} while (e.type != SDL_KEYDOWN);
		return e.key.keysym.sym;
	};

	auto maze = std::make_unique<Maze>(2000, 1200);

	constexpr double branchChance = 1.0 / 10;
	constexpr double loopChance = 0; // 1.0 / 25;
	constexpr double bridgeChance = 0.8;
	maze->generate(branchChance, loopChance, bridgeChance);

	// let's look for cycles and highlight them
	// this won't highlight every possible cycle, but if all highlighted cycles are broken then all possible cycles will also be broken.
	Cell* start = maze->getStart();
	if (start == NULL) {
		std::cerr << "no starting point?";
		return 1;
	}

	bool foundloop = false;
	std::vector<Cell*> loop;
	constexpr int paletteSize = 5;
	constexpr Uint32 palette[paletteSize] = { 0xa24a7cff, 0xfb8891ff, 0xffc094ff, 0x92ddc8ff, 0x65b2bcff };
	int loopCounter = 0;

	std::vector<Cell*> prevLinks(maze->size(), NULL);
	std::vector<int> distances(maze->size(), 0);
	auto getIndex = [&](Cell* c) -> size_t { return c - maze->data(); };
	std::function<void(Cell*, Cell*)> prevLinkEdge = [&](Cell* p, Cell* c) -> void {
		//if (foundloop)
		//	return; // don't look further
		if (prevLinks[getIndex(p)] == c)
			return; // it's the path back where we came from

		if (c->state == TraversalState::discovered)
			return;
		if (c->state == TraversalState::processed) {
			foundloop = true;
			loop.clear();
			std::vector<Cell*> pairPath;

			// handle unequal path lengths back to common vertex
			int pDist = distances[getIndex(p)];
			int cDist = distances[getIndex(c)];
			if (cDist > pDist) {
				pairPath.push_back(c);
				c = prevLinks[getIndex(c)];
				cDist--;
			}
			if (cDist < pDist) {
				loop.push_back(p);
				p = prevLinks[getIndex(p)];
				pDist--;
			}

			do {
				loop.push_back(p);
				pairPath.push_back(c);
				p = prevLinks[getIndex(p)];
				c = prevLinks[getIndex(c)];
			} while (p != c);
			loop.push_back(p);
			while (!pairPath.empty()) {
				loop.push_back(pairPath.back());
				pairPath.pop_back();
			}
			loop.push_back(loop.front());

			maze->renderThinPath(loop, palette[loopCounter%paletteSize]);
			maze->present();
			loopCounter++;
			return;
		}
		size_t index = getIndex(c);
		prevLinks[index] = p;
		distances[index] = distances[getIndex(p)] + 1;
	};
	std::function<void(Cell*)> nopVertex = [&](Cell* c) -> void {};
	maze->BFS(start, nopVertex, nopVertex, prevLinkEdge);

	// let's do a two player maze solving game
	// the players will try to find a path to each other
	constexpr Uint32 playerColors[2] = { 0xbb0000ff, 0x0000bbff };
	constexpr SDL_KeyCode keyBindings[2][5] = {
		{SDLK_RIGHT, SDLK_UP, SDLK_LEFT, SDLK_DOWN, SDLK_BACKSPACE},
		{SDLK_d, SDLK_w, SDLK_a, SDLK_s, SDLK_q}
	};
	auto getDirection = [&](int player, const SDL_Keycode key) -> int {
		for (int i = 0; i < 5; i++)
			if (key == keyBindings[player][i])
				return i;
		return -1;
	};

	std::array<std::vector<Cell*>, 2> playerPaths;
	playerPaths[0].push_back(maze->getStart());
	playerPaths[1].push_back(maze->getFinish());

	auto checkWin = [&]() -> bool {
		return std::find(playerPaths[0].begin(), playerPaths[0].end(), playerPaths[1].back()) != playerPaths[0].end() ||
			std::find(playerPaths[1].begin(), playerPaths[1].end(), playerPaths[0].back()) != playerPaths[1].end();
	};

	bool won = false;
	while (running && !won) {
		const SDL_Keycode key = waitKeyCheckQuit();

		for (int player = 0; player < 2; player++) {
			std::vector<Cell*>& path = playerPaths[player];

			auto backtrack = [&]() {
				maze->clearCell(path.back());
				path.pop_back();
				maze->clearCell(path.back());
			};

			int direction = getDirection(player, key);
			if (direction < 0)
				continue;
			if (direction == 4) {
				backtrack();
			} else {
				Cell* last = path.back();
				if (!last->connections[direction])
					break;
				Cell* next = maze->getNeighbor(last, direction, last->verticalConnections[direction]);
				if (path.size() > 1 && next == path[path.size() - 2])
					backtrack();
				else
					path.push_back(next);
				won = checkWin();
			}
			maze->renderPath(path, playerColors[player]);
			maze->present();
		}
	}

	if (won) {
		// show something rewarding, yeah?
		// TODO

		// wait for user to quit
		while (running)
			waitKeyCheckQuit();
	}

	return 0;
}

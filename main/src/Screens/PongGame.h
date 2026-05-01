#ifndef CLOCKSTAR_FIRMWARE_PONGGAME_H
#define CLOCKSTAR_FIRMWARE_PONGGAME_H

#include "../LV_Interface/LVScreen.h"
#include "../Devices/IMU.h"
#include "../Services/ChirpSystem.h"
#include "../Util/Events.h"
#include "../Util/EMA.h"
#include "../Util/Threaded.h"
#include <atomic>

class PongGame : public LVScreen {
public:
	PongGame();
	~PongGame() override;

private:
	void onStart() override;
	void onStop() override;
	void loop() override;

	// UI Objects
	lv_obj_t* bg;
	lv_obj_t* scoreLabel;
	lv_obj_t* ball;
	lv_obj_t* paddle;

	// Game State
	struct {
		float x, y;           // Ball position
		float vx, vy;         // Ball velocity
	} ballState;

	float paddleY;            // Paddle position (0-1)
	int score;
	bool gameOver;

	// Game Constants
	static constexpr int SCREEN_WIDTH = 128;
	static constexpr int SCREEN_HEIGHT = 128;
	static constexpr int BALL_SIZE = 4;
	static constexpr int PADDLE_WIDTH = 4;
	static constexpr int PADDLE_HEIGHT = 24;
	static constexpr float BALL_SPEED = 1.5f;
	static constexpr float PADDLE_SPEED = 0.02f;

	// Game Thread
	ThreadedClosure gameThread;
	std::atomic_bool running{false};
	void gameLoop();

	// IMU
	IMU* imu;
	EMA pitchFilter;
	static constexpr float filterStrength = 0.15f;

	// Services
	ChirpSystem* audio;
	EventQueue queue;

	// Game Logic
	void updateGame();
	void checkCollisions();
	void resetBall();
	void updateUI();
	void playHitSound();
	void playMissSound();

	// Input
	void handleInput();
};

#endif // CLOCKSTAR_FIRMWARE_PONGGAME_H

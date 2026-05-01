#include "PongGame.h"
#include "Util/Services.h"
#include "Devices/Input.h"
#include "MainMenu/MainMenu.h"
#include "Util/Notes.h"
#include "Services/SleepMan.h"
#include "Util/stdafx.h"
#include <cmath>
#include <algorithm>
#include <esp_random.h>

PongGame::PongGame() 
	: paddleY(0.5f),
	  score(0),
	  gameOver(false),
	  gameThread([this](){ gameLoop(); }, "Pong", 4096, 5, 1),
	  pitchFilter(filterStrength),
	  queue(4)
{
	// Get services
	imu = (IMU*) Services.get(Service::IMU);
	audio = (ChirpSystem*) Services.get(Service::Audio);

	// Create background
	bg = lv_obj_create(*this);
	lv_obj_set_size(bg, SCREEN_WIDTH, SCREEN_HEIGHT);
	lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
	lv_obj_set_style_border_width(bg, 1, 0);
	lv_obj_set_style_border_color(bg, lv_color_white(), 0);

	// Create score label
	scoreLabel = lv_label_create(bg);
	lv_label_set_text(scoreLabel, "Score: 0");
	lv_obj_set_style_text_color(scoreLabel, lv_color_white(), 0);
	lv_obj_set_pos(scoreLabel, 5, 5);

	// Create ball
	ball = lv_obj_create(bg);
	lv_obj_set_size(ball, BALL_SIZE, BALL_SIZE);
	lv_obj_set_style_bg_color(ball, lv_color_white(), 0);
	lv_obj_set_style_bg_opa(ball, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(ball, 0, 0);
	lv_obj_set_style_radius(ball, BALL_SIZE / 2, 0);

	// Create paddle
	paddle = lv_obj_create(bg);
	lv_obj_set_size(paddle, PADDLE_WIDTH, PADDLE_HEIGHT);
	lv_obj_set_style_bg_color(paddle, lv_color_white(), 0);
	lv_obj_set_style_bg_opa(paddle, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(paddle, 0, 0);

	// Initialize game state
	resetBall();
}

PongGame::~PongGame() {
	Events::unlisten(&queue);
}

void PongGame::onStart() {
	// Disable auto-sleep during game
	auto sleep = (SleepMan*) Services.get(Service::Sleep);
	sleep->enAutoSleep(false);

	// Listen for input events
	Events::listen(Facility::Input, &queue);

	// Initialize IMU filter
	IMU::Sample sample = imu->getSample();
	pitchFilter.reset(sample.accelY);

	// Start game thread
	running = true;
	gameThread.start();
}

void PongGame::onStop() {
	// Stop game thread
	running = false;
	gameThread.stop(0);

	// Cleanup
	Events::unlisten(&queue);

	// Re-enable auto-sleep
	auto sleep = (SleepMan*) Services.get(Service::Sleep);
	sleep->enAutoSleep(true);
}

void PongGame::loop() {
	// Handle button input in main UI thread
	handleInput();
}

void PongGame::resetBall() {
	ballState.x = SCREEN_WIDTH / 2;
	ballState.y = SCREEN_HEIGHT / 2;
	
	// Random angle between -45 and 45 degrees
	float angle = (esp_random() % 90 - 45) * M_PI / 180.0f;
	ballState.vx = BALL_SPEED * std::cos(angle);
	ballState.vy = BALL_SPEED * std::sin(angle);
}

void PongGame::updateGame() {
	if(gameOver) return;

	// Update ball position
	ballState.x += ballState.vx;
	ballState.y += ballState.vy;

	// Check collisions
	checkCollisions();

	// Update paddle from IMU
	IMU::Sample sample = imu->getSample();
	float pitch = pitchFilter.update(sample.accelY);
	
	// Map pitch to paddle position (-0.3g to 0.3g → 0 to 1)
	float targetY = std::clamp((pitch + 0.3f) / 0.6f, 0.0f, 1.0f);
	
	// Smooth paddle movement
	paddleY += (targetY - paddleY) * PADDLE_SPEED;
	paddleY = std::clamp(paddleY, 0.0f, 1.0f);
}

void PongGame::checkCollisions() {
	// Top/bottom walls
	if(ballState.y <= 0 || ballState.y >= SCREEN_HEIGHT - BALL_SIZE) {
		ballState.vy = -ballState.vy;
		playHitSound();
	}

	// Left wall - paddle side
	if(ballState.x <= PADDLE_WIDTH) {
		float paddleTop = paddleY * (SCREEN_HEIGHT - PADDLE_HEIGHT);
		float paddleBottom = paddleTop + PADDLE_HEIGHT;
		
		if(ballState.y >= paddleTop && ballState.y <= paddleBottom) {
			// Hit paddle
			ballState.vx = -ballState.vx;
			ballState.x = PADDLE_WIDTH;
			
			// Add spin based on hit position
			float hitPos = (ballState.y - paddleTop) / PADDLE_HEIGHT;
			ballState.vy += (hitPos - 0.5f) * 0.5f;
			
			score++;
			playHitSound();
		} else if(ballState.x <= 0) {
			// Miss - game over
			gameOver = true;
			playMissSound();
		}
	}

	// Right wall
	if(ballState.x >= SCREEN_WIDTH - BALL_SIZE) {
		ballState.vx = -ballState.vx;
		ballState.x = SCREEN_WIDTH - BALL_SIZE;
		playHitSound();
	}
}

void PongGame::updateUI() {
	// Update ball position
	lv_obj_set_pos(ball, (int)ballState.x, (int)ballState.y);

	// Update paddle position
	int paddlePixelY = paddleY * (SCREEN_HEIGHT - PADDLE_HEIGHT);
	lv_obj_set_pos(paddle, 0, paddlePixelY);

	// Update score
	char buf[32];
	if(gameOver) {
		snprintf(buf, sizeof(buf), "Score: %d\nGame Over!", score);
	} else {
		snprintf(buf, sizeof(buf), "Score: %d", score);
	}
	lv_label_set_text(scoreLabel, buf);
}

void PongGame::gameLoop() {
	const TickType_t frameTime = pdMS_TO_TICKS(16);  // ~60 FPS
	
	while(running) {
		TickType_t startTick = xTaskGetTickCount();
		
		// Update game state
		updateGame();
		
		// Update UI
		updateUI();
		
		// Maintain frame rate
		TickType_t elapsed = xTaskGetTickCount() - startTick;
		if(elapsed < frameTime) {
			vTaskDelay(frameTime - elapsed);
		}
	}
}

void PongGame::handleInput() {
	Event event;
	if(queue.get(event, 0)) {
		if(event.facility == Facility::Input) {
			auto data = (Input::Data*) event.data;
			
			if(data->action == Input::Data::Press) {
				if(data->btn == Input::Button::Alt) {
					// Exit game
					transition([](){ return std::make_unique<MainMenu>(); });
				} else if(data->btn == Input::Button::Select && gameOver) {
					// Restart game
					score = 0;
					gameOver = false;
					resetBall();
				}
			}
		}
	}
}

void PongGame::playHitSound() {
	audio->play({
		Chirp{ 
			.startFreq = NOTE_C5, 
			.endFreq = NOTE_C5, 
			.duration = 50 
		}
	});
}

void PongGame::playMissSound() {
	audio->play({
		Chirp{ .startFreq = NOTE_C4, .endFreq = NOTE_C3, .duration = 200 },
		Chirp{ .startFreq = 0, .endFreq = 0, .duration = 100 },
		Chirp{ .startFreq = NOTE_C3, .endFreq = NOTE_C2, .duration = 300 }
	});
}

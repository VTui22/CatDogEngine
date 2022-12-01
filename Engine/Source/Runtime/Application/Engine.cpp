﻿#include "Engine.h"

#include "Rendering/GBuffer.h"
#include "Rendering/RenderContext.h"
#include "Rendering/SwapChain.h"
#include "Rendering/Renderer.h"
#include "Time/Clock.h"
#include "Window/Window.h"

namespace engine
{

Engine::Engine(std::unique_ptr<IApplication> pApplication)
	: m_pApplication(std::move(pApplication))
{
}

Engine::~Engine()
{
	m_pApplication->Shutdown();
	Shutdown();
}

void Engine::Init(EngineInitArgs args)
{
	InitRenderContext(args.width, args.height);
	m_pApplication->Init(args);
}

void Engine::Run()
{
	Clock clock;

	while (true)
	{
		clock.Update();

		if (!m_pApplication->Update(clock.GetDeltaTime()))
		{
			// quit
			break;
		}
	}
}

void Engine::Shutdown()
{

}

void Engine::InitRenderContext(uint16_t width, uint16_t height)
{
	m_pRenderContext = std::make_unique<RenderContext>();
	m_pRenderContext->Init();
}

//
// Helper functions to create mutiple kinds of applications.
//
static std::unique_ptr<Engine> s_pEngine;

Engine* Engine::Create(std::unique_ptr<IApplication> pApplication)
{
	IApplication* pWhatApp = pApplication.get();
	s_pEngine = std::make_unique<Engine>(std::move(pApplication));
	pWhatApp->SetEngine(s_pEngine.get());
	return s_pEngine.get();
}

void Engine::Destroy(Engine* pEngine)
{
	assert(s_pEngine.get() == pEngine);
	if (s_pEngine)
	{
		s_pEngine->Shutdown();
		s_pEngine.reset();
		s_pEngine = nullptr;
	}
}

}
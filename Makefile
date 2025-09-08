# Makefile for robot teleoperation project

.PHONY: help install install-dev test test-unit test-integration test-fast lint format clean build

help:  ## Show this help message
	@echo "Available commands:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

install:  ## Install the package with uv
	uv sync

install-dev:  ## Install with development dependencies
	uv sync --extra dev

test:  ## Run all tests
	uv run pytest -v

test-unit:  ## Run only unit tests
	uv run pytest -v -m unit

test-integration:  ## Run only integration tests
	uv run pytest -v -m integration

test-fast:  ## Run tests excluding slow ones
	uv run pytest -v -m "not slow"

test-coverage:  ## Run tests with coverage report
	uv run pytest --cov=robot_teleop --cov-report=html --cov-report=term

lint:  ## Run linting checks
	uv run flake8 robot_teleop tests
	uv run mypy robot_teleop

format:  ## Format code with black and isort
	uv run black robot_teleop tests examples
	uv run isort robot_teleop tests examples

clean:  ## Clean up generated files
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info/
	rm -rf .pytest_cache/
	rm -rf htmlcov/
	rm -rf .coverage
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

build:  ## Build the package
	uv build

run-example:  ## Run the example test system
	uv run python test_system.py

run-monitor:  ## Run the debug monitor
	uv run python debug_monitor.py

# Quick test commands
quick-test: test-fast  ## Alias for test-fast

# Development workflow
dev-setup: install-dev  ## Set up development environment

# CI commands
ci-test: install-dev lint test-coverage  ## Run full CI test suite
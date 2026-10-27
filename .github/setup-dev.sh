#!/bin/bash
# Developer setup script for blackbox_analyzer project
# This script sets up the development environment with pre-commit hooks

set -e

echo "🚀 Setting up blackbox_analyzer development environment..."

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "❌ Error: This script must be run from the root of the git repository."
    exit 1
fi

# Install pre-commit hook
echo "📥 Installing pre-commit hook for automatic formatting and clippy checks..."
if [ -f ".github/pre-commit-hook.sh" ]; then
    cp .github/pre-commit-hook.sh .git/hooks/pre-commit
    chmod +x .git/hooks/pre-commit
    echo "✅ Pre-commit hook installed successfully."
else
    echo "❌ Error: .github/pre-commit-hook.sh not found."
    exit 1
fi

# Verify Rust toolchain
echo "🔧 Verifying Rust toolchain..."
if ! command -v cargo &> /dev/null; then
    echo "❌ Error: cargo not found. Please install Rust toolchain."
    echo "Visit: https://rustup.rs/"
    exit 1
fi

echo "✅ Rust toolchain found: $(rustc --version)"

# Run initial checks
echo "🧪 Running initial project checks..."

echo "📝 Formatting code..."
cargo fmt --all

echo "🔍 Running clippy..."
if ! cargo clippy --all-targets --all-features -- -D warnings; then
    echo "⚠️  Some clippy issues found. Please fix them before committing."
fi

echo "🏗️  Building project..."
if ! cargo build --release; then
    echo "⚠️  Build failed. Please fix issues before committing."
fi

echo ""
echo "🎉 Development environment setup complete!"
echo ""
echo "📋 Before committing, always run:"
echo "   cargo fmt --all                                 # Format code"
echo "   cargo fmt --all -- --check                      # Verify formatting"
echo "   cargo clippy --all-targets --all-features -- -D warnings  # Check for issues"
echo "   cargo test --verbose                            # Run tests"
echo "   cargo build --release                           # Build release"
echo ""
echo "🔧 The pre-commit hook will automatically format and check your code on each commit."

# SLAM Test Framework Makefile

.PHONY: proto test clean help

# Default target
help:
	@echo "SLAM Round-Trip Test Framework"
	@echo ""
	@echo "Targets:"
	@echo "  make proto    - Compile protobuf files"
	@echo "  make test     - Run test with defaults (2 min)"
	@echo "  make test-quick - Run quick test (30 sec)"
	@echo "  make test-long  - Run long test (5 min)"
	@echo "  make clean    - Clean results directory"
	@echo ""
	@echo "Manual run:"
	@echo "  ./test_slam.sh --duration 60 --output ./results/mytest"

# Compile protobuf
proto:
	@echo "Compiling protobuf..."
	protoc --python_out=scripts/proto \
		-I ../dhruva-slam/proto \
		../dhruva-slam/proto/dhruva.proto
	@echo "Done: scripts/proto/dhruva_pb2.py"

# Run tests
test:
	./test_slam.sh --duration 120

test-quick:
	./test_slam.sh --duration 30

test-long:
	./test_slam.sh --duration 300

# Clean results
clean:
	rm -rf results/*
	@echo "Cleaned results/"

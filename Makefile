train:
	docker compose -f docker-compose.yml -f docker-compose.train.yml up

test:
	docker compose -f docker-compose.yml -f docker-compose.test.yml up

jetson:
	docker compose -f docker-compose.yml -f docker-compose.jetson.yml up

viz:
	docker compose -f docker-compose.yml -f docker-compose.viz.yml up

build:
	docker compose -f docker-compose.yml build

build-jetson:
	docker compose -f docker-compose.jetson.yml build

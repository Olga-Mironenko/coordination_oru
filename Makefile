up: compose.yaml
	docker compose up --build --force-recreate --remove-orphans

compose.yaml: compose_yaml.py
	./compose_yaml.py


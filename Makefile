up: compose.yaml
	docker compose up --build --force-recreate

compose.yaml: compose_yaml.py
	./compose_yaml.py


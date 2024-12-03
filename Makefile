up: compose.yaml
	docker compose up --build --force-recreate --remove-orphans

compose.yaml: compose_yaml.py
	./compose_yaml.py

view-c1:
	vncviewer -ViewOnly localhost:5901
bash-c1:
	docker exec -it c1 bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/Lucas/Documents/codes/BlocoRobotica/FundamentosSistemasRoboticos/at/tp1_ws/install/setup.bash

echo "========================================================"
echo "1. TESTE DE DOMINIO DDS (ROS_DOMAIN_ID) E NAMESPACES"
echo "========================================================"
ROS_DOMAIN_ID=6 ros2 run comunicacao_ros2 monitor_transporte --ros-args -r __ns:=/hospital_andar1 &
PID_DOM1=$!
sleep 2

echo "--- Tentativa de comunicacao a partir do DOMAIN_ID 20 (isolamento esperado) ---"
ROS_DOMAIN_ID=20 ros2 node list

echo "--- Comunicacao restabelecida no mesmo DOMAIN_ID 6 ---"
ROS_DOMAIN_ID=6 ros2 node list

kill -9 $PID_DOM1 2>/dev/null
sleep 1

echo "========================================================"
echo "2. TESTE CLI INSPECTION: NODE INFO, TOPIC INFO VERBOSE, SERVICE LIST"
echo "========================================================"
ros2 run comunicacao_ros2 monitor_transporte &
PID_MON=$!
sleep 2

echo "--- ros2 node info /monitor_transporte ---"
ros2 node info /monitor_transporte

echo "--- ros2 topic info /alertas --verbose (evidenciando QoS RELIABLE e TRANSIENT_LOCAL) ---"
ros2 topic info /alertas --verbose

echo "--- ros2 service list ---"
ros2 service list

echo "========================================================"
echo "3. TESTE CLIENT PYTHON ASSINCRONO COM CALL_ASYNC"
echo "========================================================"
timeout 4 ros2 run comunicacao_ros2 confirmar_entrega_client

kill -9 $PID_MON 2>/dev/null
sleep 1

echo "========================================================"
echo "4. TESTE ACTION SERVER E ACTION CLIENT COM FEEDBACK E CANCELAMENTO"
echo "========================================================"
ros2 run comunicacao_ros2 transporte_action_server &
PID_ACT=$!
sleep 2

echo "--- Execucao do Action Client com recebimento de feedback e resultado ---"
timeout 7 ros2 run comunicacao_ros2 transporte_action_client

kill -9 $PID_ACT 2>/dev/null

echo "========================================================"
echo "TESTES CONCLUIDOS COM SUCESSO"
echo "========================================================"

from flask import Flask, request, jsonify
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import logging

app = Flask(__name__)
logging.basicConfig(level=logging.INFO)

@app.route('/', methods=['GET'])
def health():
    return jsonify({'status':  'OR-Tools Solver Running'}), 200

@app.route('/optimize', methods=['POST'])
def optimize():
    try:
        data = request.json
        logging.info(f"Received optimization request: {len(data. get('jobs', []))} jobs, {len(data.get('drivers', []))} drivers")
        
        jobs = data['jobs']
        drivers = data['drivers']
        time_matrix = data['time_matrix']
        
        num_vehicles = len(drivers)
        num_locations = len(time_matrix)
        
        # Routing Manager
        manager = pywrapcp. RoutingIndexManager(
            num_locations,
            num_vehicles,
            [d['start_idx'] for d in drivers],
            [d['end_idx'] for d in drivers]
        )
        
        routing = pywrapcp.RoutingModel(manager)
        
        # Time callback
        def time_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(time_matrix[from_node][to_node])
        
        time_callback_index = routing.RegisterTransitCallback(time_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)
        
        # Time dimension
        routing.AddDimension(
            time_callback_index,
            30 * 60,  # 30 dk slack
            24 * 3600,  # max route time
            False,
            'Time'
        )
        time_dimension = routing.GetDimensionOrDie('Time')
        
        # Pickup & Delivery constraints
        for job in jobs:
            pickup_idx = manager.NodeToIndex(job['pickup_node'])
            dropoff_idx = manager.NodeToIndex(job['dropoff_node'])
            
            # Time windows (±15 dakika)
            pickup_tw_start = job['pickup_time'] - 15 * 60
            pickup_tw_end = job['pickup_time'] + 15 * 60
            
            time_dimension.CumulVar(pickup_idx).SetRange(pickup_tw_start, pickup_tw_end)
            time_dimension.CumulVar(dropoff_idx).SetRange(0, 24 * 3600)
            
            # Pickup-delivery pairing
            routing.AddPickupAndDelivery(pickup_idx, dropoff_idx)
            routing.solver().Add(
                routing. VehicleVar(pickup_idx) == routing.VehicleVar(dropoff_idx)
            )
            routing.solver().Add(
                time_dimension.CumulVar(pickup_idx) <= time_dimension.CumulVar(dropoff_idx)
            )
        
        # Capacity constraint
        def demand_callback(from_index):
            node = manager.IndexToNode(from_index)
            for job in jobs:
                if node == job['pickup_node']: 
                    return job['passengers']
                if node == job['dropoff_node']:
                    return -job['passengers']
            return 0
        
        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,
            [d['capacity'] for d in drivers],
            True,
            'Capacity'
        )
        
        # Driver shift times
        for i, driver in enumerate(drivers):
            start_idx = routing.Start(i)
            end_idx = routing.End(i)
            time_dimension.CumulVar(start_idx).SetRange(driver['shift_start'], driver['shift_start'])
            time_dimension.CumulVar(end_idx).SetRange(0, driver['shift_end'])
        
        # Search parameters
        search_params = pywrapcp.DefaultRoutingSearchParameters()
        search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        search_params.time_limit. seconds = 30
        
        # Solve
        solution = routing.SolveWithParameters(search_params)
        
        if not solution:
            logging.error("No solution found")
            return jsonify({'error': 'Uygun rota bulunamadı.  Sürücü sayısını artırın veya time window ayarlarını gevşetin. '}), 400
        
        # Parse solution
        routes = []
        for vehicle_id in range(num_vehicles):
            route = []
            index = routing.Start(vehicle_id)
            
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                time_var = time_dimension.CumulVar(index)
                route.append({
                    'node': node,
                    'arrival_time': solution.Value(time_var)
                })
                index = solution.Value(routing.NextVar(index))
            
            # End node
            node = manager.IndexToNode(index)
            time_var = time_dimension.CumulVar(index)
            route.append({
                'node':  node,
                'arrival_time': solution.Value(time_var)
            })
            
            if len(route) > 2:  # Sadece ev-ev olmayan rotaları dahil et
                routes.append({
                    'driver_id': drivers[vehicle_id]['id'],
                    'driver_name': drivers[vehicle_id]['name'],
                    'driver_phone': drivers[vehicle_id]['phone'],
                    'route': route
                })
        
        logging.info(f"Solution found: {len(routes)} routes")
        return jsonify({'routes': routes, 'status': 'success'})
    
    except Exception as e: 
        logging.error(f"Error:  {str(e)}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__': 
    import os
    port = int(os.environ.get('PORT', 5000))
    app.run(host='0.0.0.0', port=port)
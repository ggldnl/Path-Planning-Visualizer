import importlib


class AlgorithmHandler:
    def handle_algorithm(self, algorithm_name):
        try:

            # Construct the full import path
            subfolder = "search_based"
            module_path = f'controllers.{subfolder}.{algorithm_name}'

            # Import the module dynamically
            algorithm_module = importlib.import_module(module_path)

            # Get the class dynamically
            algorithm_class = getattr(algorithm_module, algorithm_name)

            # Instantiate the class
            algorithm_instance = algorithm_class()

            # Execute the algorithm
            result = algorithm_instance.execute()

            return result
        except (ImportError, AttributeError) as e:
            return f"Error handling algorithm: {str(e)}"


algorithm_handler = AlgorithmHandler()
algorithm_name = "SearchAlgorithm2"
result = algorithm_handler.handle_algorithm(algorithm_name)
print(result)


import numpy as np

def distance_to_numbers(target, numbers):
    numbers = np.asarray(numbers)
    sorted_indices = np.argsort(np.abs(numbers - target))
    
    nearest_indices = sorted_indices[:2]
    nearest_values = numbers[nearest_indices]
    
    distances = np.abs(nearest_values - target)
    
    return nearest_indices, distances

# Example usage
target_number = 45
array_of_numbers = np.array([0, 120, 240])

nearest_indices, distances = distance_to_numbers(target_number, array_of_numbers)

print("Nearest indices:", nearest_indices)
print(f"Actual Numbers: {array_of_numbers[nearest_indices[0]]}, {array_of_numbers[nearest_indices[1]]}" )
print("Distances:", distances)

def distance_to_numbers(target, numbers):
    distances_and_numbers = [(num,abs(target - num)) for num in numbers]
    seg_ang_dist = sorted(distances_and_numbers, key=lambda x: x[1])
    return seg_ang_dist

# Example usage
target_number = 45
array_of_numbers = [0, 120, 240]

nearest_numbers = distance_to_numbers(target_number, array_of_numbers)


print(f"Neighbours: (Number, Distance): {nearest_numbers[0]}, {nearest_numbers[1]}" )

# print("Distances:", distances)
import cv2
import numpy as np
import os

def apply_rgb_filters(image):
    """Apply RGB color filtering (BGR in OpenCV)"""
    # Define BGR color ranges for red, green, and blue
    ranges = {
        'red': ([0, 0, 100], [100, 100, 255]), 
        'green': ([0, 100, 0], [100, 255, 100]), 
        'blue': ([100, 0, 0], [255, 100, 100])
    }

    results = {}
    for color, (lower, upper) in ranges.items():
        # Convert lists to NumPy arrays of type uint8
        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)

        # Create mask and apply bitwise_and
        mask = cv2.inRange(image, lower_np, upper_np)
        filtered = cv2.bitwise_and(image, image, mask=mask)

        results[color] = {'mask': mask, 'filtered': filtered}
    
    return results

def apply_hsv_filters(image):
    """Apply HSV color filtering"""
    # Convert image from BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV ranges for red, green, and blue
    # Note: Red needs two ranges because HSV wraps around
    ranges = {
        'red': [[np.array([0, 100, 100]), np.array([10, 255, 255])],
                [np.array([170, 100, 100]), np.array([180, 255, 255])]],
        'green': [[np.array([40, 50, 50]), np.array([80, 255, 255])]],
        'blue': [[np.array([100, 150, 0]), np.array([140, 255, 255])]]
    }

    results = {}
    for color, bounds_list in ranges.items():
        mask = None
        for lower, upper in bounds_list:
            # Create mask for current range
            current_mask = cv2.inRange(hsv, lower, upper)
            if mask is None:
                mask = current_mask
            else:
                mask = cv2.bitwise_or(mask, current_mask)

        # Extract the color region from the original image
        filtered = cv2.bitwise_and(image, image, mask=mask)
        results[color] = {'mask': mask, 'filtered': filtered}

    return results

def display_results(original, rgb_results, hsv_results):
    """Display results in separate windows"""
    cv2.imshow('Original', original)
    
    for color in ['red', 'green', 'blue']:
        # TODO: Show the filtered images for both RGB and HSV
        # Hint: use cv2.imshow()
        cv2.imshow(f'RGB Filtered - {color}', rgb_results[color]['filtered'])
        cv2.imshow(f'HSV Filtered - {color}', hsv_results[color]['filtered'])

def main():
    """Main function"""
    # Load or create image
    image_path = "images/wheel.png"
    image = cv2.imread(image_path)
    print(f"Image loaded: {image.shape}")

    # TODO: Apply both RGB and HSV filters
    rgb_results = apply_rgb_filters(image)
    hsv_results = apply_hsv_filters(image)

    # TODO: Display results
    display_results(image, rgb_results, hsv_results)
       
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test script for overtaking sector checking function
This can be used standalone to debug overtaking sector parameters
"""

import rospy
import sys


def check_overtaking_sectors():
    """Check that overtaking sectors are enabled (ot_flag set to true) for global and all cars"""
    print("🔍 Checking overtaking sectors...")

    try:
        # Define parameter namespaces to check: global + each car
        param_namespaces = ['/ot_map_params',
                            '/car1/ot_map_params', '/car2/ot_map_params']

        overall_success = True

        for namespace in param_namespaces:
            # Check if this namespace exists
            if not rospy.has_param(namespace):
                print(f"   ⚠️  {namespace} not found - skipping")
                continue

            # Get overtaking sector parameters for this namespace
            ot_params = rospy.get_param(namespace)
            n_sectors = ot_params.get('n_sectors', 0)

            if n_sectors == 0:
                print(f"   ⚠️  {namespace}: No sectors defined")
                continue

            # Display namespace being processed
            namespace_label = "Global" if namespace == '/ot_map_params' else namespace.split('/')[
                1]
            print(f"   [{namespace_label}] Found {n_sectors} overtaking sectors")

            # Check and enable each sector's ot_flag
            for i in range(n_sectors):
                sector_key = f'Overtaking_sector{i}'
                if sector_key in ot_params:
                    sector_params = ot_params[sector_key]
                    ot_flag = sector_params.get('ot_flag', False)

                    if ot_flag:
                        print(f"      ✅ Sector {i}: already enabled")
                    else:
                        print(f"      🔧 Sector {i}: DISABLED - enabling now")
                        # Update the parameter
                        rospy.set_param(
                            f'{namespace}/{sector_key}/ot_flag', True)
                else:
                    print(f"      ⚠️  Sector {i}: not found")
                    overall_success = False

        if overall_success:
            print("   ✅ All overtaking sectors checked and enabled")
        else:
            print("   ⚠️  Some issues found with overtaking sectors")

        return overall_success

    except Exception as e:
        print(f"   ❌ Error checking overtaking sectors: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main entry point"""

    print("="*70)
    print("🏎️  Overtaking Sector Test Script")
    print("="*70)
    print()

    # Initialize ROS node
    try:
        rospy.init_node('test_ot_check', anonymous=True)
        print("✅ ROS node initialized")
    except rospy.exceptions.ROSException as e:
        print(f"❌ Failed to initialize ROS node: {e}")
        print("   Make sure roscore is running!")
        sys.exit(1)

    print()

    # Run the check
    success = check_overtaking_sectors()

    print()
    print("="*70)

    if success:
        print("✅ Test completed successfully")
        sys.exit(0)
    else:
        print("⚠️  Test completed with warnings")
        sys.exit(1)


if __name__ == '__main__':
    main()

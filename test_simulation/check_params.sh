#!/bin/bash
# Check current simulation parameters

echo "=========================================="
echo "🔍 Current Simulation Parameters"
echo "=========================================="
echo

# Check simulation_id
echo "1. Simulation ID:"
sim_id=$(rosparam get /simulation_id 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "   /simulation_id = $sim_id"
else
    echo "   ⚠️  /simulation_id not set"
fi
echo

# Check simulation_complete
echo "2. Simulation Complete Flag:"
sim_complete=$(rosparam get /simulation_complete 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "   /simulation_complete = $sim_complete"
else
    echo "   ⚠️  /simulation_complete not set"
fi
echo

# Check race_complete_reason
echo "3. Race Complete Reason:"
reason=$(rosparam get /race_complete_reason 2>/dev/null)
if [ $? -eq 0 ]; then
    echo "   /race_complete_reason = $reason"
else
    echo "   ℹ️  /race_complete_reason not set (only available after race completion)"
fi
echo

# Check overtaking sectors for all three namespaces
echo "4. Overtaking Sectors:"

for namespace in "/ot_map_params" "/car1/ot_map_params" "/car2/ot_map_params"; do
    # Determine label
    if [ "$namespace" == "/ot_map_params" ]; then
        label="Global"
    else
        label=$(echo $namespace | cut -d'/' -f2)
    fi

    n_sectors=$(rosparam get ${namespace}/n_sectors 2>/dev/null)
    
    if [ $? -eq 0 ]; then
        echo "   [$label] $n_sectors sectors:"
        
        for ((i=0; i<n_sectors; i++)); do
            ot_flag=$(rosparam get ${namespace}/Overtaking_sector${i}/ot_flag 2>/dev/null)
            
            if [ "$ot_flag" == "True" ] || [ "$ot_flag" == "true" ]; then
                echo "      ✅ Sector $i: enabled"
            else
                echo "      ❌ Sector $i: disabled"
            fi
        done
    else
        echo "   ⚠️  [$label] No overtaking sectors found at $namespace"
    fi
    echo
done

echo "=========================================="

using UnityEngine;
using Unity.MLAgents;
using System.Collections.Generic;
using System.Linq;

public class LessonManager : MonoBehaviour
{
    public List<float> PalletLessons;
    public List<int> GhostLessons;

    [System.Serializable]
    public class LessonData
    {
        public float palletValue;
        public int ghostValue;
    }
    public List<LessonData> CombinedLessons;

    public int CurrentPalletIndex;
    public int CurrentGhostIndex;
    public int CurrentCombinedIndex;

    private int numberOfLessonsSucceded = 0;
    private int numberOfFailures = 0;

    StatsRecorder statsRecorder;

    void Start()
    {
        statsRecorder = Academy.Instance.StatsRecorder;

        CurrentPalletIndex = 0;
        CurrentGhostIndex = 0;
        CurrentCombinedIndex = 9;

        PalletLessons = new List<float> { 0.1f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f };
        GhostLessons = new List<int> { 0, 1, 2, 3, 4 };

        // Create combined lessons
        CombinedLessons = new List<LessonData>();
        int ghostIndex = 0;

        foreach (var pallet in PalletLessons)
        {
            CombinedLessons.Add(new LessonData { palletValue = pallet, ghostValue = GhostLessons[ghostIndex] });
            if (pallet == PalletLessons.Last())
            {
                foreach (var ghostValue in GhostLessons)
                {
                    if(ghostValue!= GhostLessons.First())
                        CombinedLessons.Add(new LessonData { palletValue = pallet, ghostValue = ghostValue });

                }

            }
        }
        statsRecorder.Add("LessonManager/CurrentCombinedIndex", CurrentCombinedIndex);
    }

    public LessonData GetCurrentCombinedValue()
    {
        return CombinedLessons[CurrentCombinedIndex];
    }

    public void AdvanceCombinedLesson()
    {
        if (!IsLastCombinedLesson())
        {
            MLAgent[] agents = FindObjectsOfType<MLAgent>();

            // Call InterruptEpisode on all agents
            foreach (MLAgent agent in agents)
            {
                agent.EpisodeInterrupted();
            }
            CurrentCombinedIndex++;
        }
        statsRecorder.Add("LessonManager/CurrentCombinedIndex", CurrentCombinedIndex);
    }

    public bool IsLastCombinedLesson()
    {
        return CurrentCombinedIndex == CombinedLessons.Count - 1;
    }

    public void AddSuccedLesson()
    {
        numberOfLessonsSucceded += 1;
        if (numberOfLessonsSucceded >= 10)
        {
            AdvanceCombinedLesson();
            numberOfLessonsSucceded = 0;
        }
    }
}
